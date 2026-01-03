# Waypoint 1 (Home):
#
# Grab your current Latitude, Longtitude and Altitude;
# set the given pieces of information as your origin
# point before starting offboard.

# Waypoint A:
#
#     - Lat:
#         47.399075
#
#     - Lon:
#         8.545180
#
#     - Alt:
#         200
#
#     - Mode:
#         Above Sea Level

# Waypoint B:
#
#     - Lat:
#         47.398814
#
#     - Lon:
#         8.546558
#
#     - Alt:
#         200
#
#     - Mode:
#         Above Sea Level

# Waypoint C:
#
#     - Lat:
#         47.397786
#
#     - Lon:
#         8.544415
#
#     - Alt:
#         200
#
#     - Mode:
#         Above Sea Level

# Point A - Point B - Point A - Point C - Home

import asyncio

from mavsdk import System
from mavsdk.offboard import PositionGlobalYaw

from modules import Checks


async def fetch_current_position(drone):
    async for position in drone.telemetry.position():
        latitude_deg = position.latitude_deg
        longitude_deg = position.longitude_deg
        absolute_altitude_m = position.absolute_altitude_m
        return latitude_deg, longitude_deg, absolute_altitude_m


async def wait_until_coordinate_reached(drone, lat, lon, altitude, yaw, mode):
    await drone.offboard.set_position_global(
        PositionGlobalYaw(lat, lon, altitude, yaw, mode)
    )

    async for position in drone.telemetry.position():
        if (
            lat - 10 < position.latitude_deg < lat + 10
            and lon - 10 < position.longitude_deg < lon + 10
            and altitude - 10 < position.absolute_altitude_m < altitude + 10
        ):
            return True


async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    lat, lon, altitude = await fetch_current_position(drone)
    mode = PositionGlobalYaw.AltitudeType(1)

    health_checks = Checks(drone)

    await health_checks.check_stable_connection()
    await health_checks.position_checks()

    print("arming the drone...")
    await drone.action.arm()

    print("drone is taking off...")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("activating offboard")
    await drone.offboard.set_position_global(
        PositionGlobalYaw(lat, lon, altitude, 0, mode)
    )
    await drone.offboard.start()

    asyncio.sleep(10)

    print("flying to waypoint A")
    await wait_until_coordinate_reached(drone, 47.399075, 8.545180, 130, 0, mode)

    print("flying to waypoint B")
    await wait_until_coordinate_reached(drone, 47.398814, 8.546558, 130, 0, mode)

    print("flying to waypoint A")
    await wait_until_coordinate_reached(drone, 47.399075, 8.545180, 130, 0, mode)

    print("flying to waypoint C")
    await wait_until_coordinate_reached(drone, 47.397786, 8.544415, 130, 0, mode)

    print("flying back to home position")
    await drone.action.return_to_launch()
    await health_checks.wait_until_landed()

    print("drone is disarming...")
    await drone.action.disarm()
    await drone.offboard.stop()


if __name__ == "__main__":
    asyncio.run(main())
