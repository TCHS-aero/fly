import asyncio
import csv
from math import cos, radians, sin, sqrt

from mavsdk import System
from mavsdk.offboard import PositionGlobalYaw, PositionNedYaw

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
    # Changing the degree of latitude and longitude to radian mode
    lat = radians(lat_deg)
    lon = radians(lon_deg)

    # the radius from the earth's center
    r = R_EARTH + alt_m

    # getting (x, y, z) coordinates for the drone
    x = r * cos(lat) * cos(lon)
    y = r * cos(lat) * sin(lon)
    z = r * sin(lat)
    print(x, y, z)
    return x, y, z


def distance_3d_lla(lat1, lon1, alt1, lat2, lon2, alt2):
    # converting the two waypoints to (x, y, z)
    x1, y1, z1 = lla_to_xyz(lat1, lon1, alt1)
    x2, y2, z2 = lla_to_xyz(lat2, lon2, alt2)

    # 3D distance
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

    # total distance for this leg (denominator)
    leg_dist = distance_3d_lla(lat1, lon1, alt_m1, lat2, lon2, alt_m2)
    if leg_dist == 0:
        print("Waypoint Progress: 100%")
        return

    async for position in drone.telemetry.position():
        Dlat, Dlon, Dalt_m = (
            position.latitude_deg,
            position.longitude_deg,
            position.relative_altitude_m,
        )
        dist_to_wp = distance_3d_lla(Dlat, Dlon, Dalt_m, lat2, lon2, alt_m2)

        # clamp so it stays between 0 and 1
        frac = max(0.0, min(1.0, 1.0 - dist_to_wp / leg_dist))
        percentage = frac * 100.0

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

    for i in range(len(mission_waypoints) - 1):
        start_wp = mission_waypoints[i]
        end_wp = mission_waypoints[i + 1]

        percentage_telemetry = asyncio.create_task(
            get_percentage_waypoint(
                original_waypoint=start_wp,
                next_waypoint=end_wp,
                drone=drone,
            )
        )

        await drone_movement(end_wp[0], end_wp[1], end_wp[2], 0, end_wp[3], drone)
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
