import json
import asyncio
from mavsdk.mission import MissionItem, MissionPlan




class Mission:
    def __init__(self, file, *, current_index = 0):
        self.file = file
        self.current_index = current_index
        self.total_waypoints = 0
        self.mission_plan = []
        self.waypoints = []


        self.parse_file()

    def parse_file(self):
        print(f"-- Parsing {self.file}")

        with open(self.file, "r") as read_file:
            self.waypoints = json.load(read_file)
            self.total_waypoints = len(self.waypoints)
            for waypoint in self.waypoints:
                
                if None not in waypoint.values():
                    continue
                
                try:
                    for item in waypoint.items():  
                        k, v = item
                        if v is None:
                            waypoint[k] = float('nan')
                except Exception as e:
                    print(e)
                   

            
            print("-- Success!")
            print(self.waypoints[0])
            return self.waypoints

    

    def get_current_waypoint(self):
        return self.waypoints[self.current_index]

    def get_waypoint(self, index):
        try:
            print("-- waypoint successfully retrived")
            return self.waypoints[index]
        except Exception:
            print("-- Invalid index, try again.")
            return None

    def advance_next_waypoint(self):
        if len(self.waypoints) == 0:
            print("-- Reached the end of the mission. No more waypoints to continue")
            return
            
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



    def append_waypoints(self, lon, lat, alt):
        self.waypoints.append(self.create_new_waypoint(lat, lon, alt))
        return self.waypoints
 
    def convert_mission_items_to_plan(self):
            self.mission_plan = []
            for item in self.waypoints:
                mission_item = MissionItem(
                    latitude_deg=item['latitude_deg'],
                    longitude_deg=item['longitude_deg'],
                    relative_altitude_m=10,
                    speed_m_s=50,
                    is_fly_through=True, #DO NOT ADD A LOITER TIME IF THIS IS SET TO TRUE
                    gimbal_pitch_deg=float('nan'),
                    gimbal_yaw_deg=float('nan'),
                    camera_action=MissionItem.CameraAction(0),
                    loiter_time_s=float('nan'),
                    camera_photo_interval_s=0.0,
                    acceptance_radius_m=20.0,
                    yaw_deg=0.0,
                    camera_photo_distance_m=0.0,
                    vehicle_action=MissionItem.VehicleAction(0)
                )
                self.mission_plan.append(mission_item)
            return self.mission_plan

#found problems:
# if you give the mission a loiter time and is fly through is true, then there is a coflicitng option:
# "Conflicting options set: fly_through=true and loiter_time>0. (mission_impl.cpp:455)" 
# and disables fly_through

# gibmal pitch and yaw somehow makes the drone oscillate back and forth for awhile when it gets to its waypoint before continuing on. 
# the reason we believe why it affects the drone causes problems is because we don't have a camera configured in the sitl.
# (what the hell? the most insignificant item screws our drone over :( we were laughing and confused at the same time when we found this)


# intergrate this with waypoints 
    async def upload_the_mission(self, drone):
        if not self.mission_plan:
            print("No mission plan")
            return
        print("uploading the mission")
        await drone.mission.upload_mission(MissionPlan(self.mission_plan))

    async def start_mission_plan(self, drone):
        await drone.mission.start_mission()

    async def check_mission_progress(self, drone):
        async for progress in drone.mission.mission_progress():
            print(progress)
            if progress.current == progress.total:
                print('mission complete')
                break

    async def set_current_mission_item(self, drone, index):
        try:
            await drone.mission.set_current_mission_item(index)
        except Exception as e:
            print(e)

    async def clear_mission(self, drone):
        await drone.mission.clear_mission()

    async def return_to_launch_after_mission_completion(self,drone,boolean): #takes effect once another missionplan is uploaded, so best to enable this THEN upload plan
        return await drone.mission.set_return_to_launch_after_mission(boolean)

    async def is_mission_finished(self, drone):
        return drone.mission.is_mission_finished()
    
    async def get_return_to_launch_after_mission(self,drone):
        return drone.mission.get_return_to_launch_after_mission()

    async def cancel_mission_download(self, drone):
        await drone.mission.cancel_mission_download()

    async def cancel_mission_upload(self, drone):
        await drone.mission.cancel_mission_upload

    async def download_mission(self, drone):
        return drone.mission.download_mission()

    async def download_mission_with_progress(self, drone):
        await drone.mission.download_mission_with_progress()

    async def pause_mission(self, drone):
        await drone.mission.pause_mission()

    async def upload_mission_with_progress(self, drone, mission):
        await drone.mission.upload_mission_with_progress(mission)


    
    
    
if __name__ == "__main__":
    miku = Mission('mission_files/mission_waypoints.json')
    miku.convert_mission_items_to_plan()
    print(miku.convert_mission_items_to_plan())
    print("/n", miku.mission_plan)
    miku.upload_the_mission()