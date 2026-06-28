import json

from importlib.resources import files
from fly.core.drone import Drone
from fly.core.mission import Mission

settings = files("fly.config").joinpath("settings.json")

def pull_data():
    try:
        with open(settings, "r") as read_file:
            data = json.load(read_file)
            return data
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"-- Error reading data from {settings}. Returning empty data.")
        print(e)
        return None
    
def update_port_data(port: str = None, history: list = None):
    try:
        existing_data = pull_data()

        if port is not None:
            existing_data["port"] = port
        if history is not None:
            existing_data["port-history"] = history

        with open(settings, "w") as write_file:
            print("-- Writing...")
            json.dump(existing_data, write_file, ensure_ascii=False, indent=4)
        print("-- Writing Success!")
    except Exception as e:
        print(e)
    
def update_mission_data(current: int = None, total: int = None):
    try:
        existing_data = pull_data()

        if current is not None:
            existing_data["current-mission-progress"] = current
        if total is not None:
            existing_data["total-mission-progress"] = total

        with open(settings, "w") as write_file:
            print("-- Writing...")
            json.dump(existing_data, write_file, ensure_ascii=False, indent=4)
    except Exception as e:
        print(e)

