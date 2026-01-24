import asyncclick as click
import json
from csv import DictReader
import asyncio
from main import Mission, Drone

states = "./states.json"
mission_file_json = "Mission File Path"
drone_instance_json = "Drone Instance"

def write_to_json(data):
    existing_data = pull_from_json() or {}
    click.echo("-- Data found" if existing_data else "-- No existing data found")

    for key, value in data.items():
        if existing_data.get(key) != value:
            click.echo(f"-- Writing {key}")
            existing_data[key] = value
            click.echo(f"-- Success!")

    with open(states, "w") as write_file:
        click.echo("-- Saving...")
        json.dump(existing_data, write_file, ensure_ascii=False, indent=4)
        click.echo("-- Success! Write completed.")

def pull_from_json():
    try:
        with open(states, "r") as read_file:
            click.echo(f"-- Pulling data from {states}")
            return json.load(read_file)
    except (FileNotFoundError, json.JSONDecodeError):
        click.echo(f"-- Error reading data from {states}. Returning empty data.")
        return {}

@click.group()
def cli():
    pass

@click.command()
@click.option("--importfile", type=click.Path(exists=True), help="The file required to run a mission. Must be a CSV.")
def upload_mission_file(importfile):
    if not importfile:
        click.echo("Please specify a filepath with the --importfile flag.")
        return

    data = pull_from_json()
    click.echo("-- Found data to import") if data else None
    click.echo("-- Mission file already detected. Overwriting...") if data.get(mission_file_json) else None

    if importfile.endswith(".csv"):
        mission = Mission(importfile)
        write_to_json({mission_file_json: importfile})
        click.echo(f"-- Mission loaded with {mission.total_waypoints} waypoints detected.")
    else:
        click.echo(f"-- Filetype not supported. Please select a CSV file.\n")
        click.echo("""File format:
        latitude,longitude,altitude
        47.399075,8.545180,45
        47.398814,8.546558,45
        47.397786,8.544415,45
        47.399195,8.546003,15
        47.397593,8.544971,50

These waypoints are mere examples, please update them with the relevant information.""")

@click.command()
def clear_data():
    if not pull_from_json():
        click.echo("-- No data to wipe.")
        return

    click.echo("-- Clearing persistent data...")
    with open(states, "w") as write_file:
        json.dump({mission_file_json: "", drone_instance_json: {}}, write_file, ensure_ascii=False, indent=4)
    click.echo("-- Data wiped, this action is irreversible.")

@click.command()
@click.option("--port", help="A UDP port to connect to the drone with.")
@click.option("--max_velocity", type=float, help="A velocity in m/s for the drone to move.")
@click.option("--alt", type=int, help="Default Takeoff Altitude in meters.")
async def config(port, max_velocity, alt):
    drone_data = pull_from_json().get(drone_instance_json, {})
    current_udpin = drone_data.get("udpin")

    if not port or not port.startswith("udpin://"):
        if current_udpin:
            click.echo(f"-- Found configured port {current_udpin}!")
            port = current_udpin
        else:
            click.echo("Supply a valid UDP port or connection. This is a required setting you must configure before moving on. Use the --port flag to specify a UDP. \n\n Example UDP:\n     udpin://0.0.0.0:14540")
            return

    alt = int(alt or 5)
    click.echo("-- Default altitude not specified, defaulting to 5 meters") if not alt else None
    max_velocity = max_velocity or 0.5
    click.echo("-- Max velocity not specified, defaulting to 0.5 m/s") if not max_velocity else None

    drone = Drone(udpin=port, default_takeoff_altitude=alt, max_velocity=max_velocity)
    save_drone(drone)

@click.command()
async def takeoff():
    drone = await load_drone()
    if not drone:
        click.echo("Can't find drone connection. Are you sure you ran the connect command?")
        return

    click.echo("-- Initiating preflight preparation...")
    await drone.connect()
    await drone.preflight_preparation()
    click.echo("-- Drone has successfully taken off.")
    save_drone(drone)

@click.command()
async def land():
    drone = await load_drone()
    if not drone:
        click.echo("Can't find drone connection. Are you sure you ran the connect command?")
        return

    click.echo("-- Initiating preflight preparation...")
    await drone.connect()
    await drone.land()
    click.echo("-- Landing...")
    save_drone(drone)

@click.command()
async def return_to_launch():
    drone = await load_drone()
    if not drone:
        click.echo("Can't find drone connection. Are you sure you ran the connect command?")
        return

    click.echo("-- Initiating preflight preparation...")
    await drone.connect()
    await drone.return_to_home()
    click.echo("-- Returning to launch...")
    save_drone(drone)

def save_drone(drone: Drone):
    write_to_json({
        drone_instance_json: {
            "takeoff_altitude": drone.takeoff_altitude,
            "udpin": drone.udpin,
            "max_velocity": drone.max_velocity
        }
    })


async def load_drone():
    drone_data = pull_from_json().get(drone_instance_json)
    if drone_data:
        drone = Drone(
            udpin=drone_data["udpin"],
            default_takeoff_altitude=drone_data.get("takeoff_altitude", 5),
            max_velocity=drone_data.get("max_velocity", 0.5)
        )
        await drone.connect()
        return drone
    return None

def load_mission():
    mission_file = pull_from_json().get(mission_file_json)
    return Mission(mission_file) if mission_file else None

@click.command()
@click.option("--right", type=float, help="Move the drone to the right at 0.5 m/s for 5 secs with a specified yaw.")
@click.option("--left", type=float, help="Move the drone to the left at 0.5 m/s for 5 secs with a specified yaw.")
@click.option("--up", type=float, help="Move the drone up at 0.5 m/s for 5 secs with a specified yaw.")
@click.option("--down", type=float, help="Move the drone down at 0.5 m/s for 5 secs with a specified yaw.")
@click.option("--forward", type=float, help="Move the drone forward at 0.5 m/s for 5 secs with a specified yaw.")
@click.option("--backward", type=float, help="Move the drone backward at 0.5 m/s for 5 secs with a specified yaw.")
async def move(right, left, up, down, forward, backward):
    drone = await load_drone()
    if not drone:
        click.echo("-- Drone instance not available. Please connect and take off first.")
        return

    velocity = pull_from_json().get(drone_instance_json, {}).get("max_velocity", 0.5)
    directions = {
        "right": right, "left": left, "up": up, "down": down, "forward": forward, "backward": backward
    }
    active_directions = {name: value for name, value in directions.items() if value or value == 0}

    if len(active_directions) != 1:
        click.echo("Specify exactly one direction with the yaw as a whole number. Example: --right 90")
        return

    direction, yaw = next(iter(active_directions.items()))
    click.echo(f"-- Moving drone {velocity} m/s {direction}...")
    try:
        move_method = getattr(drone, f"move_{direction}_offset", None)
        if move_method:
            await move_method(velocity, yaw)
    except Exception as e:
        click.echo(f"-- Error moving the drone: {e}")

    save_drone(drone)

if __name__ == "__main__":
    cli.add_command(upload_mission_file, name="upload")
    cli.add_command(clear_data, name="clear")
    cli.add_command(config, name="config")
    cli.add_command(takeoff, name="takeoff")
    cli.add_command(land, name="land")
    cli.add_command(return_to_launch, name="return")
    cli.add_command(move, name="move")
    cli()