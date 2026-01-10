import asyncio
import csv
from math import cos, radians, sin, sqrt

from mavsdk import System
from mavsdk.offboard import PositionGlobalYaw

from modules import Checks

R_EARTH = 6371000


async def collect_current_pos(drone):
    async for telemetry in drone.telemetry.position():
        lat = telemetry.latitude_deg
        lon = telemetry.longitude_deg
        alt_m = telemetry.relative_altitude_m
        return lat, lon, alt_m


async def mission_poi():
    mission_waypoints = []
    with open("./mission_waypoints.csv") as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        for row in reader:
            if row[0].startswith("//"):
                continue
            mission_waypoints.append(
                [
                    float(row[0]),
                    float(row[1]),
                    float(row[2]),
                    PositionGlobalYaw.AltitudeType(int(row[3])),
                ]
            )

    number_of_waypoints = len(mission_waypoints)
    return (mission_waypoints, number_of_waypoints)


def lla_to_xyz(lat_deg, lon_deg, alt_m):
    lat = radians(lat_deg)
    lon = radians(lon_deg)

    r = R_EARTH + alt_m

    x = r * cos(lat) * cos(lon)
    y = r * cos(lat) * sin(lon)
    z = r * sin(lat)
    print("debug test_ Math")
    print(x, y, z)
    return x, y, z


def distance_3d_lla(lat1, lon1, alt1, lat2, lon2, alt2):
    x1, y1, z1 = lla_to_xyz(lat1, lon1, alt1)
    x2, y2, z2 = lla_to_xyz(lat2, lon2, alt2)

    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    print(dx, dy, dz)
    print("debug test_ Distance 3d lla")
    return sqrt(dx * dx + dy * dy + dz * dz)


async def get_percentage_waypoint(original_waypoint, next_waypoint, drone):
    lat1, lon1, alt_m1 = (
        original_waypoint[0],
        original_waypoint[1],
        original_waypoint[2],
    )
    lat2, lon2, alt_m2 = (
        next_waypoint[0],
        next_waypoint[1],
        next_waypoint[2],
    )
    print("debug test, get percentage waypoint1")
    async for position in drone.telemetry.position():
        Dlat, Dlon, Dalt_m = (
            position.latitude_deg,
            position.longitude_deg,
            position.relative_altitude_m,
        )
        num = distance_3d_lla(
            lat2=Dlat, lon2=Dlon, alt2=Dalt_m, lat1=lat1, lon1=lon1, alt1=alt_m1
        )
        den = distance_3d_lla(
            lat2=lat2, lon2=lon2, alt2=alt_m2, lat1=lat1, lon1=lon1, alt1=alt_m1
        )
        percentage = 1 - (num / den)
        print("after percentage")
        print("debug test, get percentage waypoint3")
        print(percentage, "% of the way there waypoint-wise")
    print("For loop over")


async def drone_movement(lat, lon, alt_m, yaw, flight_mode, drone):
    print(f"-- Moving to {lat}, {lon}!")
    await drone.offboard.set_position_global(
        PositionGlobalYaw(lat, lon, alt_m, yaw, flight_mode)
    )

    async for position in drone.telemetry.position():
        if (
            (position.latitude_deg - 0.000001 < lat < position.latitude_deg + 0.000001)
            and (
                position.longitude_deg - 0.000001
                < lon
                < position.longitude_deg + 0.000001
            )
            and (
                position.relative_altitude_m - 0.5
                < alt_m
                < position.relative_altitude_m + 0.5
            )
        ):
            print("drone reached waypoint")
            break

    await asyncio.sleep(5)


async def main():
    drone = System()
    health_check = Checks(drone)
    mode = PositionGlobalYaw.AltitudeType(0)
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    mission_waypoints, number_of_waypoints = await mission_poi()
    await health_check.check_stable_connection()
    await health_check.position_checks()

    print("arming the drone...")
    await drone.action.arm()

    print("drone is taking off...")
    await drone.action.set_takeoff_altitude(45.0)
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("setting home coords...")
    lat, lon, alt_m = await collect_current_pos(drone)
    await drone.offboard.set_position_global(
        PositionGlobalYaw(lat, lon, alt_m, 0, mode)
    )
    home = [lat, lon, alt_m, mode]
    mission_waypoints.extend([home])

    print("drone is activating offboard...")
    await drone.offboard.start()

    await asyncio.sleep(1)

    for i in range(len(mission_waypoints)):
        wp = mission_waypoints[i]
        if i + 1 >= len(mission_waypoints):
            percentage_telemetry = asyncio.create_task(
                get_percentage_waypoint(
                    mission_waypoints[-1],
                    home,
                    drone,
                )
            )
        else:
            percentage_telemetry = asyncio.create_task(
                get_percentage_waypoint(
                    mission_waypoints[-1],
                    wp,
                    drone,
                )
            )
        await drone_movement(wp[0], wp[1], wp[2], 0, wp[3], drone)
        print(
            f"drone is {i + 1}/{number_of_waypoints + 1} including the home coordinate"
        )
        await asyncio.sleep(3)
        percentage_telemetry.cancel()

    await drone.action.land()

    await health_check.wait_until_landed()

    await drone.action.disarm()
    print("drone 100% completed their mission.")


if __name__ == "__main__":
    asyncio.run(main())
