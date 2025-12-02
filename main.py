import asyncio  # asyncio module for threading
import math

from mavsdk import System  # importing system class from mavsdk liibbrary
from mavsdk.offboard import OffboardError, PositionNedYaw


async def main():
    """
    Main logic for the drone.
    Takeoff, landing, telemetry.
    """

    drone = System()
    await drone.connect(
        system_address="udpin://0.0.0.0:14540"
    )  # connects script to udp port to communicate with physical drone

    # get_yaw = asyncio.create_task(get_yaw(drone))

    print("-- Waiting for drone to connect...")
    # checks to see if drone is connected and has a stable connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Drone discovered!")
            break

    print("-- Waiting for drone to have a global position estimate...")
    # checks to see if position is calibrated and stable
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    try:
        await drone.action.arm()
    except Exception as e:
        print(e)

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(
            f"Starting offboard mode failed \
                with error code: {error._result.result}"
        )
        print("-- Disarming")
        await drone.action.disarm()
        return

    # finds yaw of the drone
    # asyncio.ensure_future(get_yaw(drone))
    # asyncio.ensure_future(get_position(drone))

    print("-- Taking Off")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0, -3, 0))

    await asyncio.sleep(15)
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 10.0, -3, 180.0))
    await asyncio.sleep(5)
    await drone.offboard.set_position_ned(PositionNedYaw(5.0, 10.0, -3, 90.0))
    await asyncio.sleep(10)
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -3, 0.0))
    await asyncio.sleep(10)
    await drone.offboard.stop()
    await drone.action.land()


async def get_yaw(_drone):
    async for altitude_euler in _drone.telemetry.attitude_euler():
        yaw = altitude_euler
        print(f"yaw of drone is {yaw} degrees")


async def get_position(_drone):
    R = 6371000  # this is the radius of the earth in meters
    async for position in _drone.telemetry.position():
        position_x = (
            R
            * math.cos(math.radians(position.latitude_deg))
            * math.cos(math.radians(position.longitude_deg))
        )
        position_y = (
            R
            * math.cos(math.radians(position.latitude_deg))
            * math.sin(math.radians(position.longitude_deg))
        )
        position_z = R * math.sin(math.radians(position.latitude_deg))
        print(
            f"positionX = {position_x} positionY = {position_y} positionZ = {position_z}"
        )


if __name__ == "__main__":
    asyncio.run(main())
