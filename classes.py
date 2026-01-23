import csv
import asyncio
from mavsdk import System
from mavsdk.offboard import PositionGlobalYaw

class Mission:
    def __init__(self, file):
        self.file = file
        self.current_index = 0
        self.waypoints = []

        self.parse_file()

    def parse_file(self):
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

    def get_current_waypoint(self):
        return self.waypoints[self.current_index]

    def get_waypoint(self, index):
        try:
            return self.waypoints[index]
        except:
            print("invalid index...try again...")
            return None

    def advance_next_waypoint(self):
        if len(self.waypoints) == 0:
            print("no waypoints to travel to.")
        self.current_index += 1
        try:
            return self.waypoints[self.current_index]
        except:
            print("index is out of range!")

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
                    "-- No new data to assign to keys or too many data per column. Defaulting to None..."
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
    

    def __init__(self, default_takeoff_altitude = 5):
        self.drone = System()
        self.takeoff_altitude = default_takeoff_altitude

    async def preflight_preparation(self):
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Found stable connecton")
                break
            print("Stable connection not found, trying again...")

        async for health_check in self.drone.telemetry.health():
            if health_check.is_global_position_ok and health_check.is_home_position_ok:
                print("Health check completed") 
                break
            print("Health checks failed, retrying...")
        await self.drone.action.arm()
        print('arming the drone')
        await self.drone.action.set_takeoff_altitude(self.takeoff_altitude)
        await self.drone.action.takeoff()

    async def current_position(self):
        async for telemetry in self.drone.telemetry.position():
            self.lat_deg = telemetry.latitude_deg
            self.lon_deg = telemetry.longitude_deg
            self.abs_altitude_m = telemetry.absolute_altitude_m
            self.rel_altitude_m= telemetry.relative_altitude_m
            print(f'Latitude: {self.lat_deg}\nLongitude: {self.lon_deg}\nAbsolute Altitude from Sea Level: {self.abs_altitude_m}\nRelative Altitude from Ground: {self.rel_altitude_m}')
            await asyncio.sleep(2)            

    async def move_to_location(self, lat, lon, alt, yaw, mode):
        await self.drone.offboard.set_position_global(
            PositionGlobalYaw(lat, lon, alt, yaw, mode)
            )
        async for position in self.drone.telemetry.position():
            if (
                (position.latitude_deg - 0.000001 < position.latitude_deg < position.latitude_deg + 0.000001
                ) and (
                position.longitude_deg - 0.000001 < position.longitude_deg < position.longitude_deg + 0.000001                
                ) and (
                position.relative_altitude_m - 0.5 < position.relative_altitude_m < position.relative_altitude_m + 0.5
                )
            ):
                print('drone has arrived to the checkpoint! checkpoint!')
                break


    async def move_left_offset(self, offset, yaw):
        self.north_m, self.east_m, self.down_m = self.drone.telemetry.PositionNed()
        return await self.drone.offboard.PositionNedYaw(self.north_m, (self.east_m - offset), self.down_m, yaw)

    async def move_right_offset(self, offset, yaw):
        self.north_m, self.east_m, self.down_m = self.drone.telemetry.PositionNed()
        return await self.drone.offboard.PositionNedYaw(self.north_m, (self.east_m + offset), self.down_m, yaw)

    async def move_down_offset(self, offset, yaw):
        self.north_m, self.east_m, self.down_m = self.drone.telemetry.PositionNed()
        return await self.drone.offboard.PositionNedYaw(self.north_m , self.east_m, (self.down_m + offset), yaw)
        
    async def move_up_offset(self, offset, yaw):
        self.north_m, self.east_m, self.down_m = self.drone.telemetry.PositionNed()
        return await self.drone.offboard.PositionNedYaw(self.north_m, self.east_m, (self.down_m - offset), yaw)
    
    async def evade(self):
        pass
        
    async def move_to_waypoint(self, waypoint):
        await self.drone.Mission.set_current_mission_item(waypoint)
        pass

    async def pause_mission(self):
        await self.drone.Mission.pause_mission()

    async def resume_mission(self):
        await self.drone.Mission.start_mission()

    async def return_to_home(self):
        await self.drone.Mission.set_return_to_launch_after_mission()

    async def fetch_drone_instance(self):
        return self.drone

async def main():
    droneclass = Drone()
    mission = Mission("mission_waypoints.csv")
    
    await droneclass.preflight_preparation()
    current_pos = asyncio.create_task(await droneclass.current_position())
    await droneclass.move_to_location(mission.get_current_waypoint(1), 0, 0)
    await droneclass.return_to_home()
    await current_pos.cancel()
    

    

if __name__ == "__main__":
    asyncio.run(main())