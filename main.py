import csv
import asyncio
import click
from mavsdk import System
from mavsdk.offboard import PositionGlobalYaw, PositionNedYaw

class Mission:
    def __init__(self, file):
        self.file = file
        self.current_index = 0
        self.waypoints = []
        self.total_waypoints = 0

        self.parse_file()

    def parse_file(self):
        print(f"-- Parsing {self.file}")
        with open(self.file) as csvfile:
            reader = csv.DictReader(csvfile)

            for row in reader:
                for key, value in row.items():
                    try:
                        if "." in value:
                            row[key] = float(value)
                        elif value.isdigit():
                            row[key] = int(value)
                    except:
                        pass
                self.waypoints.append(row)
                self.total_waypoints = len(self.waypoints)
            
        print(f"-- Success!")


    def get_current_waypoint(self):
        return self.waypoints[self.current_index]

    def get_waypoint(self, index):
        try:
            return self.waypoints[index]
        except:
            print("-- Invalid index, try again.")
            return None

    def advance_next_waypoint(self):
        if len(self.waypoints) == 0:
            print("-- Reached the end of the mission. No more waypoints to continue")
        self.current_index += 1
        try:
            return self.waypoints[self.current_index]
        except Exception:
            print("-- Invalid index, try again.")

    def reset_mission(self):
        self.current_index = 0
        return self.get_current_waypoint()

    def select_waypoint(self, index):
        try:
            self.current_index = index
            return self.get_current_waypoint()
        except Exception as e:
            print(f"-- {e}")
            return None

    def get_keys(self):
        return list(self.waypoints[-1].keys())

    def create_new_waypoint(self, lat, lon, alt):
        data = (lat, lon, alt)
        new_waypoint = {}
        keys = self.get_keys()
        for i in range(len(keys)):
            try:
                new_waypoint[keys[i]] = data[i]
            except Exception:
                print(
                    "-- No new data to assign to keys or too much data per column. Defaulting to None..."
                )
                new_waypoint[keys[i]] = None

        return new_waypoint

    def insert_waypoints(self, lon, lat, alt, index):
        self.waypoints.insert(index, self.create_new_waypoint(lat, lon, alt))
        return self.waypoints

    def replace_waypoints(self, lon, lat, alt, index):
        self.waypoints[index] = self.create_new_waypoint(lat, lon, alt)
        return self.waypoints

    def append_waypopints(self, lon, lat, alt):
        self.waypoints.append(self.create_new_waypoint(lat, lon, alt))
        return self.waypoints

class Drone():
    def __init__(self, udpin = "udpin://0.0.0.0:14540", default_takeoff_altitude = 5):
        self.drone = System()
        self.takeoff_altitude = default_takeoff_altitude
        self.mode = PositionGlobalYaw.AltitudeType(0)
        self.udpin = udpin

    async def connect(self):
        await self.drone.connect(system_address=self.udpin)
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Found stable connecton")
                break
            print("-- Stable connection not found, retrying...")

    async def preflight_preparation(self):
        
        async for health_check in self.drone.telemetry.health():
            if health_check.is_global_position_ok and health_check.is_home_position_ok:
                print("-- Health check completed") 
                break
            print("-- Health checks failed, retrying...")

        await self.drone.action.arm()
        print('-- Armed')
        await self.drone.action.set_takeoff_altitude(self.takeoff_altitude)
        print("-- Setting home coordinates...")

        await self.drone.action.takeoff()

        await asyncio.sleep(5)

    async def current_position(self):
        async for telemetry in self.drone.telemetry.position():
            lat_deg = telemetry.latitude_deg
            lon_deg = telemetry.longitude_deg
            rel_altitude_m= telemetry.relative_altitude_m
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
                (position.latitude_deg - 0.000001 < lat < position.latitude_deg + 0.000001
                ) and (
                position.longitude_deg - 0.000001 < lon < position.longitude_deg + 0.000001                
                ) and (
                position.relative_altitude_m - 0.5 < alt < position.relative_altitude_m + 0.5
                )
            ):
                print('-- Successfully reached checkpoint')
                break

        await self.drone.offboard.stop()

    async def current_ned(self):
        async for telemetry in self.drone.telemetry.position_velocity_ned():
            ned_object = telemetry.position
            north_m = ned_object.north_m
            east_m = ned_object.east_m
            down_m = ned_object.down_m
            return north_m, east_m, down_m
        

    async def move_left_offset(self, offset, yaw=0):
        north, east, down = await self.current_ned()
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(north, east, down, 0)
        )
        await self.drone.offboard.start()
        await asyncio.sleep(1)
        await self.drone.offboard.set_position_ned( 
            PositionNedYaw(north, east - float(offset), down, float(yaw))
            )
        
        await asyncio.sleep(5)
        await self.drone.offboard.stop()

    async def move_right_offset(self, offset, yaw = 0):
        north, east, down = await self.current_ned()
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(north, east, down, 0)
        )
        await self.drone.offboard.start()
        await asyncio.sleep(1)
        await self.drone.offboard.set_position_ned( 
            PositionNedYaw(north, east + offset, down, float(yaw))
            )
        
        await asyncio.sleep(5)
        await self.drone.offboard.stop()

    async def move_down_offset(self, offset, yaw = 0):
        north, east, down = await self.current_ned()
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(north, east, down, 0)
        )
        await self.drone.offboard.start()
        await asyncio.sleep(1)
        await self.drone.offboard.set_position_ned( 
            PositionNedYaw(north, east, down + offset, float(yaw))
            )
        
        await asyncio.sleep(5)
        await self.drone.offboard.stop()

    async def move_up_offset(self, offset, yaw = 0):
        north, east, down = await self.current_ned()
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(north, east, down, 0)
        )
        await self.drone.offboard.start()
        await asyncio.sleep(1)
        await self.drone.offboard.set_position_ned( 
            PositionNedYaw(north, east, down - offset, float(yaw))
            )
        
        await asyncio.sleep(5)
        await self.drone.offboard.stop()
    
    async def move_forward_offset(self, offset, yaw = 0):
        north, east, down = await self.current_ned()
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(north, east, down, 0)
        )
        await self.drone.offboard.start()
        await asyncio.sleep(1)
        await self.drone.offboard.set_position_ned( 
            PositionNedYaw(north + offset, east, down, float(yaw))
            )
        
        await asyncio.sleep(5)
        await self.drone.offboard.stop()
        
    async def move_backward_offset(self, offset, yaw = 0):
        north, east, down = await self.current_ned()
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(north, east, down, 0)
        )
        await self.drone.offboard.start()
        await asyncio.sleep(1)
        await self.drone.offboard.set_position_ned( 
            PositionNedYaw(north - offset, east, down, float(yaw))
            )
        
        await asyncio.sleep(5)
        await self.drone.offboard.stop()

    async def evade(self):
        pass
        
    async def move_to_waypoint(self, dict, yaw = 0.0):
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
                (position.latitude_deg - 0.000001 < lat < position.latitude_deg + 0.000001
                ) and (
                position.longitude_deg - 0.000001 < lon < position.longitude_deg + 0.000001                
                ) and (
                position.relative_altitude_m - 0.5 < alt < position.relative_altitude_m + 0.5
                )
            ):
                print('-- Successfully reached waypoint')
                break
        
        await self.drone.offboard.stop()

    async def return_to_home(self):
        await self.drone.action.return_to_launch()

    async def land(self):
        await self.drone.action.land()

    async def fetch_drone_instance(self):
        return self.drone
