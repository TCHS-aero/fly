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
                    return 1
            
            print("-- Success!")
            return self.waypoints

    def get_all_waypoints(self):
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

    def get_keys(self):
        return list(self.waypoints[-1].keys())

    def convert_mission_items_to_plan(self):
            self.mission_plan = []
            for item in self.waypoints:
               mission_item = MissionItem(
                    **item
               )
               self.mission_plan.append(mission_item)
            return self.mission_plan

    def advance_next_waypoint(self):
        if len(self.waypoints) <= 0:
            print("-- There are no waypoints loaded into this mission.")
            return

        self.current_index += 1
        try:
            return self.waypoints[self.current_index]
        except Exception:
            print("-- Invalid index, try again.")

    async def return_to_launch_after_mission_completion(self, drone, boolean):
        return await drone.mission.set_return_to_launch_after_mission(boolean)

    async def reset_mission(self):

    async def upload_mission(self, drone, return_to_launch):
        self.convert_mission_items_to_plan()
        await self.return_to_launch_after_mission_completion(drone, return_to_launch)
        await drone.mission.upload_mission(MissionPlan(self.mission_plan))

    async def start_mission(self, drone):
        await drone.mission.start_mission()

    async def check_mission_progress(self, drone):
        async for progress in drone.mission.mission_progress():
            return progress
            if progress.current == progress.total:
                print('mission complete')
                break

    async def set_current_mission_target(self, drone, index):
        try:
            await drone.mission.set_current_mission_item(index)
        except Exception as e:
            print(e)

    async def clear_mission(self, drone):
        await drone.mission.clear_mission()

    async def is_mission_finished(self, drone):
        return drone.mission.is_mission_finished()
    
    async def get_return_to_launch_after_mission(self,drone):
        return drone.mission.get_return_to_launch_after_mission()

    async def cancel_mission_download(self, drone):
        await drone.mission.cancel_mission_download()

    async def cancel_mission_upload(self, drone):
        await drone.mission.cancel_mission_upload()

    async def download_mission(self, drone):
        mission = await drone.mission.download_mission()

    async def download_mission_with_progress(self, drone):
        mission = await drone.mission.download_mission_with_progress()

    async def pause_mission(self, drone):
        await drone.mission.pause_mission()

    async def upload_mission_with_progress(self, drone, mission):
        self.convert_mission_items_to_plan()
        await self.return_to_launch_after_mission_completion(drone, return_to_launch)
        await drone.mission.upload_mission_with_progress(MissionPlan(self.mission_plan))
