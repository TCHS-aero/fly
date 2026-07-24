# connect/status, takeoff, land, RTL

import asyncclick as click

from fly.interfaces.cli.session import get_connected_drone, require_drone


@click.group(help="Connect to and manually fly the drone (connect, takeoff, land, move).")
def flight():
    pass


@flight.command(help="Connect to a drone and remember the port for future commands.")
@click.option(
    "--port",
    help=("udp(in|out)://host:port, tcp(in|out)://host:port, or serial://dev/tty...",
          "Defaults to the last-used port, or udpin://0.0.0.0:14540")
)
@click.option("--timeout", type=int, default=10, show_default=True, help="Connection timeout in seconds.")
async def connect(port, timeout):
    # uses get_connected_drone() rather than require_drone() since
    # it reports success/failure
    drone = await get_connected_drone(port, timeout=timeout)
    if not drone:
        raise SystemExit(1)
    print("-- Connected. Port saved as default for future commands.")

@flight.command(help="Report the drone's current position, heading, and flight mode.")
@click.option("--port", help="Connection port. Defaults to the last-used port.")
async def status(port):
    drone = await require_drone(port)

    pos = await drone.current_position()
    if pos is None:
        print("-- Current position unavailable.")
        raise SystemExit(1)
    lat, lon, alt_rel = pos
    heading = await drone.current_heading()
    speed = await drone.current_ground_speed()
    print(f"-- lat={lat:.7f} lon={lon:.7f} rel_alt={alt_rel:.2f}m")
    print(f"-- heading={heading:.1f} deg   ground_speed={speed:.2f} m/s")

@flight.command(help="Command the drone to take off to a specified altitude.")
@click.option("--port", help="Connection port. Defaults to the last-used port.")
@click.option("--alt", type=float, default=5.0, show_default=True, help="Takeoff altitude in meters.")
async def takeoff(port, alt):
    if alt <= 0:
        print("-- Altitude must be positive meters.")
        raise SystemExit(1)

    drone = await require_drone(port)

    print("-- Preparing for takeoff...")
    await drone.takeoff(alt)

@flight.command(help="Land the drone at its current position.")
@click.option("--port", help="Connection port. Defaults to the last-used port.")
async def land(port):
    drone = await require_drone(port)
    await drone.land()
    print("-- Landing initiated.")

@flight.command(name="return", help="Return to the launch (home) position and land.")
@click.option("--port", help="Connection port. Defaults to the last-used port.")
async def return_to_launch(port):
    drone = await require_drone(port)
    await drone.return_to_home()
    print("-- Returning to launch position.")

@click.group(help="Manual NED-frame nudges (left/right/up/down/forward/backward/stop).")
def move():
    pass

async def _execute_movement(direction, port, velocity, yaw, distance):
    if velocity <= 0 or distance <= 0:
        print("velocity and distance must be greater than 0.")
        raise SystemExit(1)

    drone = await require_drone(port)

    print(f"-- Moving {direction} at {velocity} m/s for {distance}m (yaw={yaw})...")
    move_method = getattr(drone, f"move_{direction}_offset", None)
    if move_method is None:
        print(f"-- No move handler for direction {direction!r}.")
        raise SystemExit(1)

    try:
        await move_method(velocity, distance, yaw=yaw)
        print("-- Move complete.")
    except Exception as e:
        print(f"-- Error moving {direction}: {e}")
        raise SystemExit(1) from e


def _move_command(direction: str):
    @click.option("--port", help="Connection port. Defaults to the last-used port.")
    @click.option("--velocity", type=float, required=True, help="Speed in m/s.")
    @click.option("--distance", type=float, required=True, help="Distance in meters.")
    @click.option("--yaw", type=float, default=0.0, show_default=True, help="Yaw angle in degrees.")
    async def _cmd(port, velocity, distance, yaw):
        await _execute_movement(direction, port, velocity, yaw, distance)

    return _cmd

_MOVE_DIRECTIONS = {
    "left": "Move negative East in the NED frame.",
    "right": "Move positive East in the NED frame.",
    "up": "Move negative Down in the NED frame.",
    "down": "Move positive Down in NED frame.",
    "forward": "Move positive North in the NED frame.",
    "backward": "Move negative North in the NED frame."
}
for _direction, _help in _MOVE_DIRECTIONS.items():
    move.command(name=_direction, help=_help)(_move_command(_direction))

@move.command(name="stop", help="Zero out velocity.")
@click.option("--port", help="Connection port. Defaults to the last-used port.")
async def stop(port):
    drone = await require_drone(port)
    await drone.stop_movement()
    print("-- Stopped")
