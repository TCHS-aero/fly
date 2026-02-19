import asyncclick as click
import json
import re
from fly.core.drone import Drone
from fly.core.mission import Mission
from fly.core.dataManager import (
    write_to_json,
    pull_from_json,
    load_drone,
    drone_instance_json,
    mission_file_json,
    wipe_config,
)


@click.group()
def cli():
    pass


@click.group(
    help="Manage all drone missions, including importing a mission and starting/stopping it."
)
def mission():
    pass


@click.group(
    help="Control the drone's movement in the North-East-Down (NED) coordinate system."
)
def move():
    pass


@click.command(
    help="Clear all current configuration and reset all settings saved in the system."
)
def wipe():
    wipe_config()


@click.command(
    help="Connect to a drone using the specified port (UDP, TCP, or Serial). Default is udpin://0.0.0.0:14540."
)
@click.option(
    "--port",
    help="A port to connect to the drone. Valid formats include UDP (e.g., udpin://0.0.0.0:14540), TCP, or Serial.",
)
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
            print(
                "-- Invalid port format. Please specify a valid UDP, TCP, or Serial port."
            )
            return

    print("-- Testing for a stable connection")
    drone = Drone(port)
    if not await drone.connect():
        print("-- Connection test failed; consider trying a different port.")
        return 1

    write_to_json(
        {
            drone_instance_json: {
                "port": port,
            }
        }
    )


@click.command(help="Command the drone to take off to a specified altitude.")
@click.option(
    "--alt", help="The altitude to take off to, in meters. Defaults to 5 meters."
)
async def takeoff(alt):
    if alt is not None:
        try:
            alt = float(alt)
            if alt < 0:
                raise TypeError("Value below 0.")
        except TypeError:
            print(
                "Please use a valid, positive floating-point value (e.g., 5.0). Altitude values below 0 are not allowed."
            )
            return

    drone = await load_drone()
    if not drone:
        return

    if not alt:
        print("-- Altitude not specified, defaulting to 5 meters.")
        alt = 5

    await drone.connect()

    print("-- Preparing for takeoff...")
    await drone.takeoff(alt)


@click.command(help="Land the drone at its current position.")
async def land():
    drone = await load_drone()
    if not drone:
        return

    await drone.connect()
    await drone.land()
    print("-- Drone landing initiated.")


@click.command(help="Land the drone at its return-to-launch (RTL) position.")
async def return_to_launch():
    drone = await load_drone()
    if not drone:
        return

    await drone.connect()
    await drone.return_to_home()
    print("-- Drone returning to launch position.")


@move.command(name="left", help="Move the drone left (negative East) in the NED frame.")
@click.option(
    "--velocity", type=float, required=True, help="The velocity in m/s (e.g., 0.5)."
)
@click.option(
    "--yaw", type=float, default=0.0, help="The yaw angle in degrees (default: 0)."
)
@click.option(
    "--distance",
    type=float,
    required=True,
    help="Movement in meters (default: 5).",
)
async def left(velocity, yaw, distance):
    await execute_movement("left", velocity, yaw, distance)


@move.command(name="right", help="Move the drone right (ast) in the NED frame.")
@click.option(
    "--velocity", type=float, required=True, help="The velocity in m/s (e.g., 0.5)."
)
@click.option(
    "--yaw", type=float, default=0.0, help="The yaw angle in degrees (default: 0)."
)
@click.option(
    "--distance",
    type=float,
    required=True,
    help="Movement in meters (default: 5).",
)
async def right(velocity, yaw, distance):
    await execute_movement("right", velocity, yaw, distance)


@move.command(name="up", help="Move the drone up (negative Down) in the NED frame.")
@click.option(
    "--velocity", type=float, required=True, help="The velocity in m/s (e.g., 0.5)."
)
@click.option(
    "--yaw", type=float, default=0.0, help="The yaw angle in degrees (default: 0)."
)
@click.option(
    "--distance",
    type=float,
    required=True,
    help="Movement in meters (default: 5).",
)
async def up(velocity, yaw, distance):
    await execute_movement("up", velocity, yaw, distance)


@move.command(name="down", help="Move the drone down (positive Down) in the NED frame.")
@click.option(
    "--velocity", type=float, required=True, help="The velocity in m/s (e.g., 0.5)."
)
@click.option(
    "--yaw", type=float, default=0.0, help="The yaw angle in degrees (default: 0)."
)
@click.option(
    "--distance",
    type=float,
    required=True,
    help="Movement in meters (default: 5).",
)
async def down(velocity, yaw, distance):
    await execute_movement("down", velocity, yaw, distance)


@move.command(
    name="forward", help="Move the drone forward (positive North) in the NED frame."
)
@click.option(
    "--velocity", type=float, required=True, help="The velocity in m/s (e.g., 0.5)."
)
@click.option(
    "--yaw", type=float, default=0.0, help="The yaw angle in degrees (default: 0)."
)
@click.option(
    "--distance",
    type=float,
    required=True,
    help="Movement in meters (default: 5).",
)
async def forward(velocity, yaw, distance):
    await execute_movement("forward", velocity, yaw, distance)


@move.command(
    name="backward", help="Move the drone backward (negative North) in the NED frame."
)
@click.option(
    "--velocity", type=float, required=True, help="The velocity in m/s (e.g., 0.5)."
)
@click.option(
    "--yaw", type=float, default=0.0, help="The yaw angle in degrees (default: 0)."
)
@click.option(
    "--distance",
    type=float,
    required=True,
    help="Movement in meters (default: 5).",
)
async def backward(velocity, yaw, distance):
    await execute_movement("backward", velocity, yaw, distance)


@move.command(
    name="stop",
    help="Stop current movement of the drone if the movement method is offboard.",
)
async def stop():
    drone = await load_drone()
    if not drone:
        return

    await drone.connect()
    await drone.stop_movement()


async def execute_movement(direction, velocity, yaw, distance):
    print(
        f"-- Preparing to move the drone {direction} at {velocity} m/s with a yaw of {yaw} degrees..."
    )

    if velocity < 0 or distance < 0:
        print("All values must be greater than 0.")
        return

    drone = await load_drone()
    if not drone:
        return

    await drone.connect()

    try:
        move_method = getattr(drone, f"move_{direction}_offset", None)
        if move_method:
            await move_method(velocity, distance, yaw=yaw)
        else:
            print("-- Movement function not found.")
    except Exception as e:
        print(f"-- Error moving the drone {direction}: {e}")


@mission.command(
    name="import",
    help="Upload a mission file in JSON format.",
)
@click.option(
    "--file",
    type=click.Path(exists=True),
    help="The JSON file defining the mission waypoints.",
)
def upload_mission_file(file):
    if not file:
        print("Please specify a filepath with the --file flag.")
        return

    data = pull_from_json()
    if data:
        if data.get("mission_file_json"):
            print("-- Mission file already detected. Overwriting...")

    if file.endswith(".json"):
        with open(file, "r") as json_file:
            data = json.load(json_file)

            if not data:
                print("The file is valid but contains an empty list or dictionary.")
                return

            keys = {"latitude", "longitude", "altitude"}
            if not all(set(item) == keys for item in data):
                print("-- Invalid JSON keys.")
                print("""Please ensure your JSON file contains the following:
        Example:
        [
            {
                "latitude": 47.399075,
                "longitude": 8.545180,
                "altitude": 45
            },
            {
                "latitude": 47.398814,
                "longitude": 8.546558,
                "altitude": 45
            }
        ]

These waypoints are mere examples, please update them with the relevant information.""")
                return

        try:
            mission = Mission(file)
            write_to_json({mission_file_json: {"file_path": file}})
            print(
                f"-- Mission loaded with {mission.total_waypoints} waypoints detected."
            )
        except Exception as e:
            print(f"-- Failed to load mission: {e}")
            return

    else:
        print("-- Filetype not supported. Please select a JSON file.\n")
        print("""File format:
        [
            {
                "latitude": 47.399075,
                "longitude": 8.545180,
                "altitude": 45
            },
            {
                "latitude": 47.398814,
                "longitude": 8.546558,
                "altitude": 45
            }
        ]

These waypoints are mere examples, please update them with the relevant information.""")


def main():
    cli.add_command(connect, name="connect")
    cli.add_command(takeoff, name="takeoff")
    cli.add_command(land, name="land")
    cli.add_command(return_to_launch, name="return")
    cli.add_command(mission_file_json, name="mission")
    cli.add_command(wipe, name="wipe_config")
    cli.add_command(mission)
    cli.add_command(move)
    cli()


if __name__ == "__main__":
    main()
