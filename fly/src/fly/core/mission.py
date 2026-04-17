import json
import asyncio
from mavsdk import System

class Mission:
    def __init__(self, file, *, current_index = 0):
        self.file = file
        self.current_index = current_index
        self.waypoints = []
        self.total_waypoints = 0
        self.mission_items = []
        self.parse_file()

    def parse_file(self):
        print(f"-- Parsing {self.file}")

        if self.file.endswith(".json"):
            with open(self.file, "r") as read_file:
                self.waypoints = json.load(read_file)
                self.total_waypoints = len(self.waypoints)

                
                print("-- Success!")
                return self.waypoints
        else:
            print("parse_file_mission.py (most likely invalid file format)")
            return None

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

    async def start_the_mission(self):
        try:
            print("teto")
            for waypoint in range(len(self.mission.waypoints)):
                print('diabeto')
                i = self.mission.get_waypoint(waypoint)
                print("dance")
                await self.drone.move_to_waypoint(self.mission.advance_next_waypoint())
                print('cookie')
        except Exception as e:
            self.log(str(e))
            print(e)


    def append_waypoints(self, lon, lat, alt):
        self.waypoints.append(self.create_new_waypoint(lat, lon, alt))
        return self.waypoints
 
 

    


    def upload_mission_plan(self):
        if not self.waypoints:
            print("no waypoint is uploaded")
        MissionPlan.upload_mission(self.mission_items)
            

    def start_mission_plan(self):
        return self.mission.start_mission()
        
        
    def clear_mission_plan(self):
        return self.mission.clear_mission()
    
    def pause_mission(self):
        if self.mission.start_mission():
            try:
                print('Mission paused.')
                return self.drone.mission.pause_mission()
            except Exception as e:
                print(f"Failed to PAUSE mission: {e}")
            

    def cancel_mission(self):
        if self.drone is None:
            print('Drone not connected')
            return false
        try:
            self.drone.mission.cancel_mission_upload()
            self.drone.mission.clear_mission()
            
            print('Mission cancelled.')
            return True
        except Exception as e:
            print(f"Failed to CANCEL mission: {e}")
            return False

    def clear_mission(self):
        try:
            self.drone.mission.clear_mission()
        except Exception as e:
            print(f"Failed to CLEAR mission: {e}")

    def is_mission_finished(self):
        try:
            check_status = self.drone.mission.is_mission_finished()
            if(check_status ):
                print('Mission finished')
        except Exception as e:
            print(f"Failed to CHECK mission: {e}")

    def convert_mission_items_to_plan(self, waypoints):
        # format: 
        # latitude_deg, longitude_deg, relative_altitude_m,
        # speed_m_s, is_fly_through, gimbal_pitch_deg, gimbal_yaw_deg, camera_action, 
        # loiter_time_s, camera_photo_interval_s, acceptance_radius_m, yaw_deg,
        # camera_photo_distance_m, vehicle_action

        # [ [lon1, lat1, alt1], [lon2, lat2, alt2] ]

        '''print(waypoints)
        try:
            for items in waypoints:
                print(items)
                break

        except Exception as e:
            print("convert mission items to plan, error message --> ", e)'''

        self.mission_plan = []
        for items in waypoints:
            self.mission_plan.append(items)
        return self.mission_plan

    
if __name__ == "__main__":
    miku = Mission("/home/lemmonboys/arrow/fly/mission_files/mission_waypoints.json")
    print(miku.convert_mission_items_to_plan(miku.waypoints))