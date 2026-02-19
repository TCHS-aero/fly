import asyncio

from mavsdk import System
from mavsdk.offboard import PositionGlobalYaw, VelocityNedYaw


class Drone:
    def __init__(self, port, *, connection_timeout=10, velocity=0.5):
        self.drone = System()
        self.mode = PositionGlobalYaw.AltitudeType(0)
        self.port = port
        self.velocity = velocity
        self.connection_timeout = connection_timeout

    async def connect(self):
        connected = False
        try:
            async with asyncio.timeout(self.connection_timeout):
                await self.drone.connect(system_address=self.port)
                async for state in self.drone.core.connection_state():
                    if state.is_connected:
                        print("-- Found a stable connection to the drone!")
                        connected = True
                        break
        except asyncio.TimeoutError:
            print(
                f"-- Failed to connect to the drone within {self.connection_timeout} seconds."
            )

        return connected

    async def takeoff(self, alt):
        async for health_check in self.drone.telemetry.health():
            if health_check.is_global_position_ok and health_check.is_home_position_ok:
                print("-- Health check completed")
                break
            print("-- Health checks failed, retrying...")

        await self.drone.action.arm()
        print("-- Armed")
        await self.drone.action.set_takeoff_altitude(alt)
        print("-- Setting home coordinates...")

        await self.drone.action.takeoff()
        print("-- Sent takeoff request!")

    async def current_position(self):
        async for telemetry in self.drone.telemetry.position():
            lat_deg = telemetry.latitude_deg
            lon_deg = telemetry.longitude_deg
            rel_altitude_m = telemetry.relative_altitude_m
            return lat_deg, lon_deg, rel_altitude_m

    async def move_to_location(self, lat, lon, alt, yaw):
        clat, clon, calt = await self.current_position()
        await self.drone.offboard.set_position_global(
            PositionGlobalYaw(clat, clon, calt, 0, self.mode)
        )
        await self.drone.offboard.start()

        await self.drone.offboard.set_position_global(
            PositionGlobalYaw(lat, lon, alt, yaw, self.mode)
        )

        async for position in self.drone.telemetry.position():
            if (
                (
                    position.latitude_deg - 0.000001
                    < lat
                    < position.latitude_deg + 0.000001
                )
                and (
                    position.longitude_deg - 0.000001
                    < lon
                    < position.longitude_deg + 0.000001
                )
                and (
                    position.relative_altitude_m - 0.5
                    < alt
                    < position.relative_altitude_m + 0.5
                )
            ):
                print("-- Successfully reached checkpoint")
                break

        await self.drone.offboard.stop()

    async def current_ned(self):
        async for telemetry in self.drone.telemetry.position_velocity_ned():
            ned_object = telemetry.position
            north_m = ned_object.north_m
            east_m = ned_object.east_m
            down_m = ned_object.down_m
            return (north_m, east_m, down_m)

    async def stop_movement(self):
        north, east, down = await self.current_ned()
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await self.drone.offboard.start()
        await asyncio.sleep(1)
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0, 0.0, 0))
        await asyncio.sleep(1)
        await self.drone.offboard.stop()

    async def move_left_offset(self, velocity, distance, *, yaw=0):
        _, east, _ = await self.current_ned()
        end_distance = east - distance
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await self.drone.offboard.start()
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, velocity * -1, 0.0, yaw)
        )
        while end_distance <= east:
            _, east, _ = await self.current_ned()
            await asyncio.sleep(0.2)
        await self.drone.offboard.stop()

    async def move_right_offset(self, velocity, distance, *, yaw=0):
        _, east, _ = await self.current_ned()
        end_point = east + distance
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await self.drone.offboard.start()
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, velocity, 0.0, yaw)
        )
        while end_point >= east:
            _, east, _ = await self.current_ned()
            await asyncio.sleep(0.2)
        await self.drone.offboard.stop()

    async def move_down_offset(self, velocity, distance, *, yaw=0):
        _, _, down = await self.current_ned()
        end_point = down + distance
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await self.drone.offboard.start()
        await asyncio.sleep(1)
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, velocity, yaw)
        )
        while end_point >= down:
            _, _, down = await self.current_ned()
            await asyncio.sleep(0.2)
        await self.drone.offboard.stop()

    async def move_up_offset(self, velocity, distance, *, yaw=0):
        _, _, down = await self.current_ned()
        end_point = down - distance
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await self.drone.offboard.start()
        await asyncio.sleep(1)
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, velocity * -1, yaw)
        )
        while end_point <= down:
            _, _, down = await self.current_ned()
            await asyncio.sleep(0.2)
        await self.drone.offboard.stop()

    async def move_forward_offset(self, velocity, distance, *, yaw=0):
        north, _, _ = await self.current_ned()
        end_point = north + distance
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await self.drone.offboard.start()
        await asyncio.sleep(1)
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(velocity, 0.0, 0.0, yaw)
        )
        while end_point >= north:
            north, _, _ = await self.current_ned()
            await asyncio.sleep(0.2)
        await self.drone.offboard.stop()

    async def move_backward_offset(self, velocity, distance, *, yaw=0):
        north, _, _ = await self.current_ned()
        end_point = north - distance
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await self.drone.offboard.start()
        await asyncio.sleep(1)
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(velocity * -1, 0.0, 0.0, yaw)
        )
        while end_point <= north:
            north, _, _ = await self.current_ned()
            await asyncio.sleep(0.2)
        await self.drone.offboard.stop()

    async def move_to_waypoint(self, dict, yaw=0.0):
        lat, lon, alt = dict.values()
        print(lat, lon, alt, yaw, self.mode)
        clat, clon, calt = await self.current_position()
        await self.drone.offboard.set_position_global(
            PositionGlobalYaw(clat, clon, calt, 0, self.mode)
        )
        await self.drone.offboard.start()

        await self.drone.offboard.set_position_global(
            PositionGlobalYaw(lat, lon, alt, yaw, self.mode)
        )

        async for position in self.drone.telemetry.position():
            if (
                (
                    position.latitude_deg - 0.000001
                    < lat
                    < position.latitude_deg + 0.000001
                )
                and (
                    position.longitude_deg - 0.000001
                    < lon
                    < position.longitude_deg + 0.000001
                )
                and (
                    position.relative_altitude_m - 0.5
                    < alt
                    < position.relative_altitude_m + 0.5
                )
            ):
                print("-- Successfully reached waypoint")
                break

        await self.drone.offboard.stop()

    async def return_to_home(self):
        await self.drone.action.return_to_launch()

    async def land(self):
        await self.drone.action.land()

    async def fetch_drone_instance(self):
        return self.drone
