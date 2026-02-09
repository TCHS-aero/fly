import json

class Mission:
    def __init__(self, file, *, current_index = 0):
        self.file = file
        self.current_index = current_index
        self.waypoints = []
        self.total_waypoints = 0

        self.parse_file()

    def parse_file(self):
        print(f"-- Parsing {self.file}")

        with open(self.file, "r") as read_file:
            self.waypoints = json.load(read_file)
            self.total_waypoints = len(self.waypoints)

        print("-- Success!")

    def get_current_waypoint(self):
        return self.waypoints[self.current_index]

    def get_waypoint(self, index):
        try:
            return self.waypoints[index]
        except Exception:
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
