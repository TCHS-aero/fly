import json
import asyncio
from mavsdk.mission import MissionItem, MissionPlan

class Mission:
    def __init__(self, file):
        self.file = file
        self.total_waypoints = 0
        self.mission_plan = []
        self.waypoints = []

        self.parse_file()

    def parse_file(self):
        print(f"-- Parsing {self.file}")

        with open(self.file, "r") as read_file:
            self.waypoints = json.load(read_file)
            self.total_waypoints = len(self.waypoints)

            self.RTL = waypoint['RTL']
            print(self.RTL)

            for waypoint in self.waypoints:
                
                if None not in waypoint.values():
                    continue
                
                try:
                    for item in waypoint.items():  
                        k, v = item
                        print(k,v,item)

                        if v is None:
                            waypoint[k] = float('nan')

                except Exception as e:
                    print(e)
                    return 
            
            print("-- Success!")
            return self.waypoints

    def get_raw_waypoints(self):
        return self.waypoints

    async def get_current_waypoint(self, drone_instance):
        if self.mission_plan == False:
            print("-- No mission uploaded")
            return

        plan = await self.download_mission(drone_instance)
        current_progress = await self.get_mission_progress(drone_instance)
        if current_progress is None:
            print("-- Mission Completed")
            return 0

        return plan.mission_items[current_progress]
    
    def get_keys(self):
        return list(self.waypoints[-1].keys())

    def convert_mission_items_to_plan(self):
            self.mission_plan = []
            for item in self.waypoints:
               mission_item = MissionItem(
                    latitude_deg=item['latitude_deg'],
                    longitude_deg=item['longitude_deg'],
                    relative_altitude_m=item['relative_altitude_m'],
                    speed_m_s=item['speed_m_s'],
                    is_fly_through=item['is_fly_through'],
                    gimbal_pitch_deg=item['gimbal_pitch_deg'],
                    gimbal_yaw_deg=item['gimbal_yaw_deg'],
                    camera_action=MissionItem.CameraAction(item['camera_action']),
                    loiter_time_s=item['loiter_time_s'],
                    camera_photo_interval_s=item['camera_photo_interval_s'],
                    acceptance_radius_m=item['acceptance_radius_m'],
                    yaw_deg=item['yaw_deg'],
                    camera_photo_distance_m=item['camera_photo_distance_m'],
                    vehicle_action=MissionItem.VehicleAction(item['vehicle_action'])
               )
               self.mission_plan.append(mission_item)
            return self.mission_plan

    async def return_to_launch_after_mission_completion(self, drone_instance, boolean):
        return await drone_instance.drone.mission.set_return_to_launch_after_mission(boolean)

    async def reset_mission(self, drone_instance):
        await self.set_current_mission_target(drone_instance, 0)

    async def upload_mission(self, drone_instance):
        self.convert_mission_items_to_plan()
        await drone_instance.drone.mission.upload_mission(MissionPlan(self.mission_plan)) 

    async def start_mission(self, drone_instance):
        await drone_instance.drone.mission.start_mission()

    async def get_mission_progress(self, drone_instance):
        async for progress in drone_instance.drone.mission.mission_progress():
            if progress.current == progress.total:
                print('-- Mission Complete!')
                return None
                break
            return progress.current

    async def set_current_mission_target(self, drone_instance, index):
        await drone_instance.drone.mission.set_current_mission_item(index)

    async def clear_mission(self, drone_instance):
        await drone_instance.drone.mission.clear_mission()

    async def is_mission_finished(self, drone_instance):
        return drone_instance.drone.mission.is_mission_finished()
    
    async def get_return_to_launch_after_mission(self, drone_instance):
        return drone_instance.drone.mission.get_return_to_launch_after_mission()

    async def cancel_mission_download(self, drone_instance):
        await drone_instance.drone.mission.cancel_mission_download()

    async def cancel_mission_upload(self, drone_instance):
        return await drone_instance.drone.mission.cancel_mission_upload()

    async def download_mission(self, drone_instance):
        return await drone_instance.drone.mission.download_mission()

    async def download_mission_with_progress(self, drone_instance):
        return await drone_instance.drone.mission.download_mission_with_progress()

    async def pause_mission(self, drone_instance):
        await drone_instance.drone.mission.pause_mission()

    async def upload_mission_with_progress(self, drone_instance, mission):
        self.convert_mission_items_to_plan()
        await self.return_to_launch_after_mission_completion(drone_instance, return_to_launch)
        await drone_instance.drone.mission.upload_mission_with_progress(MissionPlan(self.mission_plan))