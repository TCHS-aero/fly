import asyncclick as click
import json
from csv import DictReader
import asyncio
from main import Mission, Drone

states = "./states.json"
mission_file_json = "Mission File Path"
drone_instance_json = "Drone Instance"

def write_to_json(data):
    if ex := pull_from_json():
        click.echo("-- Data found")
        for key, value in ex.items():
            if key in data:
                click.echo(f"-- Writing {key}")
                ex[key] = data[key]
                click.echo(f"-- Success!")

    with open(states, "w") as write_file:
        click.echo("-- Saving...")
        json.dump(ex, write_file)
        click.echo("-- Success! Write completed.")

def pull_from_json():
    click.echo(f"-- Pulling data from {states}")
    with open(states, "r") as write_file:
            data = json.load(write_file)
            click.echo(f"-- Success!")
            return data

@click.group()
def cli():
    pass

@click.command()
@click.option("--importfile", type=click.Path(exists=True), help="The file required to run a mission. Must be a CSV.")
def upload_mission_file(importfile):
    if not importfile:
        click.echo("Please specify a filepath with the --importfile flag.")
        return

    global mission
    data = pull_from_json()
    if data:
        click.echo("-- Found data to import")
        click.echo("-- Mission file already detected. Overwriting...")
    if importfile.endswith(".csv"):
        mission = Mission(importfile)
        write_to_json({mission_file_json: importfile})
        click.echo(f"-- Mission loaded with {mission.total_waypoints} waypoints detected.")
    else:
        click.echo(f"-- Filetype not supported. Please select a CSV file.\n")
        click.echo(f"""File format:
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
        json.dump({mission_file_json: "", drone_instance_json: ""}, write_file)
        click.echo("-- Data wiped, this action is irreversible.")

@click.command()
@click.option("--port", help="A UDP port to connect to the drone with.")
@click.option("--alt", help="Default Takeoff Altitude in meters.")
async def connect(port, alt):
    if not port or not port.startswith("udpin://"):
        click.echo("Supply a valid UDP port or connection. Required to run the drone. \n\n Example UDP:\n     udpin://0.0.0.0:14540")
        return

    if not alt:
        click.echo("-- Default altitude not specified, defaulting to 5 meters")
        alt = 5
    alt = int(alt)

    drone = Drone(udpin=port, default_takeoff_altitude=alt)
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
    drone_data = {
        "takeoff_altitude": drone.takeoff_altitude,
        "udpin": drone.udpin
    }
    write_to_json({drone_instance_json: drone_data})


async def load_drone():
    data = pull_from_json()
    drone_data = data.get(drone_instance_json)
    if drone_data:
        drone = Drone(udpin=drone_data.get("udpin"), default_takeoff_altitude=drone_data.get("takeoff_altitude", 5))
        await drone.connect()
        return drone
    return None

def load_mission():
    data = pull_from_json()
    mission_file = data.get(mission_file_json, None)
    if mission_file:
        mission = Mission(mission_file)
        return mission
    return None


@click.command()
@click.option("--right", type=float, default=0, help="Move the drone to the right by the specified offset.")
@click.option("--left", type=float, default=0, help="Move the drone to the left by the specified offset.")
@click.option("--up", type=float, default=0, help="Move the drone up by the specified offset.")
@click.option("--down", type=float, default=0, help="Move the drone down by the specified offset.")
@click.option("--forward", type=float, default=0, help="Move the drone forward by the specified offset.")
@click.option("--backward", type=float, default=0, help="Move the drone backward by the specified offset.")
async def move(right, left, up, down, forward, backward):

    drone = await load_drone()

    if not drone:
        click.echo("-- Drone instance not available. Please connect and take off first.")
        return

    directions = [("right", right), ("left", left), ("up", up), ("down", down), ("forward", forward), ("backward", backward)]
    active_movements = [(name, int(value)) for name, value in directions if value > 0]

    if len(active_movements) != 1:
        click.echo("Specify exactly one direction with a positive offset. Example: --right 5")
        return

    direction, offset = active_movements[0]
    click.echo(f"-- Moving drone {offset} units {direction}...")

    try:
        if direction == "right":
            await drone.move_right_offset(offset, 0)
        elif direction == "left":
            await drone.move_left_offset(offset, 0)
        elif direction == "up":
            await drone.move_up_offset(offset, 0)
        elif direction == "down":
            await drone.move_down_offset(offset, 0)
        elif direction == "forward":
            await drone.move_forward_offset(offset, 0)
        elif direction == "backward":
            await drone.move_backward_offset(offset, 0)
    except Exception as e:
        click.echo(f"-- Error moving the drone: {e}")
    
    save_drone(drone)

if __name__ == "__main__":
    cli.add_command(upload_mission_file, name="upload")
    cli.add_command(clear_data, name="clear")
    cli.add_command(connect, name="connect")
    cli.add_command(takeoff, name="takeoff")
    cli.add_command(land, name="land")
    cli.add_command(return_to_launch, name="return")
    cli.add_command(move, name="move")
    cli()