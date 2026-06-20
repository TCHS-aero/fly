import json
import asyncio
from pathlib import Path
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.telemetry import FlightMode

class Mission:
    def __init__(self, *, file = None):
        self.file = file
        self.total_waypoints = 0
        self.mission_plan = []
        self.waypoints = []
        self.path = Path(self.file)
        self.RTL = False
        self.downloaded_plan = None

        if self.file:
            self.parse_file(self.file)

    def parse_file(self, file):
        self.file = file
        self.path = Path(file)
        print(f"-- Parsing {file}")

        with open(file, "r") as read_file:
            self.data = json.load(read_file)
            if len(self.data) < 3:
                print("-- Your mission have have more than one waypoint!")
                return

            self.RTL = self.data[0]
            self.waypoints = self.data[1:]


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
                    return 
            
            print("-- Success!")
            return self.waypoints

    def get_raw_waypoints(self):
        return self.waypoints

    async def get_current_next_waypoint_info(self, drone_instance, current_progress):
        self.downloaded_plan = await self.download_mission(drone_instance)

        if current_progress is None or current_progress < 0 or current_progress >= len(self.downloaded_plan.mission_items):
            return (self.downloaded_plan.mission_items[current_progress - 1]), None

        if current_progress == 0:
            return (None, self.downloaded_plan.mission_items[current_progress])

        try:
            return (self.downloaded_plan.mission_items[current_progress - 1], self.downloaded_plan.mission_items[current_progress])
        except Exception:
            return (self.downloaded_plan.mission_items[current_progress], None)

    async def drone_have_mission(self, drone_instance):
        self.downloaded_plan = await self.download_mission(drone_instance)

        if len(list(self.downloaded_plan.mission_items)) > 1:
            return True
        return False

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
        await self.clear_mission(drone_instance)
        self.convert_mission_items_to_plan()
        if self.mission_plan:
            await drone_instance.drone.mission.upload_mission(MissionPlan(self.mission_plan)) 
            return
        print("-- No mission to upload")

    async def start_mission(self, drone_instance):
        await drone_instance.drone.mission.start_mission()

    async def get_mission_progress(self, drone_instance):
        async for progress in drone_instance.drone.mission.mission_progress():
            return progress.current, progress.total

    async def set_current_mission_target(self, drone_instance, index):
        await drone_instance.drone.mission.set_current_mission_item(index)

        self.RTL = False
    async def clear_mission(self, drone_instance):
        await drone_instance.drone.mission.clear_mission()

        self.downloaded_plan = None
        self.mission_plan = []
        self.total_waypoints = 0

    async def is_mission_finished(self, drone_instance):
        return await drone_instance.drone.mission.is_mission_finished()
    
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

    async def upload_mission_with_progress(self, drone_instance):
        self.convert_mission_items_to_plan()
        await self.return_to_launch_after_mission_completion(drone_instance, return_to_launch)
        await drone_instance.drone.mission.upload_mission_with_progress(MissionPlan(self.mission_plan))

    async def is_drone_on_mission(self, drone_instance):
        async for current_mode in drone_instance.drone.telemetry.flight_mode():
            if current_mode == FlightMode.MISSION:
                return True
            return False   
    

