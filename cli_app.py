import asyncclick as click
import re
import json
from csv import DictReader
import asyncio
from main import Mission, Drone

settings = "./settings.json"
mission_file_json = "Mission File Path"
drone_instance_json = "Drone"

def write_to_json(data):
    existing_data = pull_from_json() or {}

    for key, value in data.items():
        if existing_data.get(key) != value:
            print(f"-- Updating {key}")
            existing_data[key] = value

    with open(settings, "w") as write_file:
        print("-- Writing...")
        json.dump(existing_data, write_file, ensure_ascii=False, indent=4)
        print("-- Writing finished")

def pull_from_json():
    try:
        with open(settings, "r") as read_file:
            return json.load(read_file)
    except (FileNotFoundError, json.JSONDecodeError):
        print(f"-- Error reading data from {settings}. Returning empty data.")
        return {}

async def load_drone():
    drone_data = pull_from_json().get(drone_instance_json)
    if drone_data:
        port = drone_data.get("port", None)
        if not port:
            print(f"-- Port has not yet been configured in {settings}. Are you sure you ran the connect command?")
            return

        drone = Drone(
            port
        )
        return drone
    return

def load_mission():
    mission_file = pull_from_json().get(mission_file_json)
    return Mission(mission_file) if mission_file else None

@click.group()
def cli():
    pass

@click.group(help="Manage drone mission operations.")
def mission():
    pass

@click.command(help="Update the port configuration; specify --port for a custom port.")
@click.option("--port", help="A port to connect to the drone with. Can be a UDP, TCP, or Serial port.")
async def connect(port):

    if not port:
        print("-- Port not specified, defaulting to udpin://0.0.0.0:14540")
        port = "udpin://0.0.0.0:14540"
    else:
        udp_pattern = r"^udp(?:in|out)?://([0-9]{1,3}\.){3}[0-9]{1,3}:[0-9]{1,5}$"
        tcp_pattern = r"^tcp(?:in|out)?://([0-9]{1,3}\.){3}[0-9]{1,3}:[0-9]{1,5}$"
        serial_pattern = r"^serial://(/dev/[a-zA-Z0-9_-]+|COM[0-9]+)(:[0-9]+)?$"

        if re.match(udp_pattern, port):
            print("-- Valid UDP port detected.")
        elif re.match(tcp_pattern, port):
            print("-- Valid TCP port detected.")
        elif re.match(serial_pattern, port):
            print("-- Valid Serial port detected.")
        else:
            print("-- Invalid port format. Please specify a valid UDP, TCP, or Serial port.")
            return

    print("-- Testing for a stable connection")
    drone = Drone(port)
    if not await drone.connect():
        print("-- Connection test failed; consider trying a different port.")
        return 1

    write_to_json({
        drone_instance_json: {
            "port": port,
        }
    })

@click.command(help="Drone takeoff; specify --alt to choose departure height.")
@click.option("--alt", help="An altitude, in meters, to takeoff from. Defaults to 5 meters.")
async def takeoff(alt):

    if alt is not None:
        try:
            alt = float(alt)
            if alt < 0:
                raise TypeError("Value below 0.")
        except TypeError:
            print(f"Please use a valid, positive, floating type integer. Pretty sure you can't fly {alt} meters up.")
            return

    drone = await load_drone()
    if not drone:
        return

    if not alt:
        print("-- Altitude not specified, defaulting to 5 meters.")
        alt = 5

    await drone.connect()

    print("-- Preparing takeoff...")
    await drone.takeoff(alt)

@click.command(help="Land the drone!!")
async def land():
    drone = await load_drone()
    if not drone:
        return

    await drone.connect()
    await drone.land()
    print("-- Request to land sent")

@click.command(help="Land at launch position from any given location.")
async def return_to_launch():
    drone = await load_drone()
    if not drone:
        return

    await drone.connect()
    await drone.return_to_home()
    print("-- Request to land sent")

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
        print("-- Drone instance not available. Please connect and take off first.")
        return

    velocity = pull_from_json().get(drone_instance_json, {}).get("max_velocity", 0.5)
    directions = {
        "right": right, "left": left, "up": up, "down": down, "forward": forward, "backward": backward
    }
    active_directions = {name: value for name, value in directions.items() if value or value == 0}

    if len(active_directions) != 1:
        print("Specify exactly one direction with the yaw as a whole number. Example: --right 90")
        return

    direction, yaw = next(iter(active_directions.items()))
    print(f"-- Moving drone {velocity} m/s {direction}...")
    try:
        move_method = getattr(drone, f"move_{direction}_offset", None)
        if move_method:
            await move_method(velocity, yaw)
    except Exception as e:
        print(f"-- Error moving the drone: {e}")

    save_drone(drone)

@mission.command(name="import", help="Upload a CSV file to run a mission from.")
@click.option("--file", type=click.Path(exists=True), help="The file required to run a mission. Must be a CSV.")
def upload_mission_file(file):
    if not file:
        print("Please specify a filepath with the --file flag.")
        return

    data = pull_from_json()
    if data:
        if data.get("mission_file_json"):
            print("-- Mission file already detected. Overwriting...")

    if file.endswith(".csv"):
        with open(file, "r") as csvfile:
            header = csvfile.readline().strip() or "null"

            pattern = r"^\s*latitude\s*,\s*longitude\s*,\s*altitude\s*$"

            if not re.match(pattern, header, re.IGNORECASE):
                print(f"-- Invalid CSV header: {header}")
                print("""Please ensure your CSV file contains the following columns:
                latitude,longitude,altitude

                Example:
                latitude,longitude,altitude
                47.399075,8.545180,45
                47.398814,8.546558,45
                47.397786,8.544415,45
                47.399195,8.546003,15
                47.397593,8.544971,50

These waypoints are mere examples, please update them with the relevant information.""")
                return

        try:
            mission = Mission(file)
            write_to_json({mission_file_json: file})
            print(f"-- Mission loaded with {mission.total_waypoints} waypoints detected.")
        except Exception as e:
            print(f"-- Failed to load mission: {e}")
            return

    else:
        print(f"-- Filetype not supported. Please select a CSV file.\n")
        print("""File format:
        latitude,longitude,altitude
        47.399075,8.545180,45
        47.398814,8.546558,45
        47.397786,8.544415,45
        47.399195,8.546003,15
        47.397593,8.544971,50

These waypoints are mere examples, please update them with the relevant information.""")


if __name__ == "__main__":
    cli.add_command(connect, name="connect")
    cli.add_command(takeoff, name="takeoff")
    cli.add_command(land, name="land")
    cli.add_command(return_to_launch, name="return")
    cli.add_command(mission_file_json, name="mission")
    cli.add_command(mission)
    cli()