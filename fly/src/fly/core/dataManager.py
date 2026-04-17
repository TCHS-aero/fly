import json

from importlib.resources import files
from fly.core.drone import Drone
from fly.core.mission import Mission

settings = files("fly.config").joinpath("settings.json")
mission_file_json = "Mission"
drone_instance_json = "Drone"


def write_to_json(data):
    existing_data = pull_from_json() or {}

    for key, value in data.items():
        if existing_data.get(key) != value:
            print(f"-- Updating {key}")
            existing_data[key] = value

    with open(settings, "w") as write_file:
        print("-- Writing...")
        json.dump(existing_data, write_file, ensure_ascii=False, indent=4)
        print("-- Writing finished")


def pull_from_json():
    try:
        with open(settings, "r") as read_file:
            return json.load(read_file)
    except (FileNotFoundError, json.JSONDecodeError):
        print(f"-- Error reading data from {settings}. Returning empty data.")
        return {}


async def load_drone():
    drone_data = pull_from_json().get(drone_instance_json)
    if drone_data:
        port = drone_data.get("port", None)
        if not port:
            print(
                f"-- Port has not yet been configured in {settings}. Are you sure you ran the connect command?"
            )
            return
 
        drone = Drone(port)
        return drone
    return


async def load_mission():
    mission = pull_from_json().get(mission_file_json)
    if mission:
        file_path = mission.get("file_path", None)
        if not file_path:
            print(
                "-- No mission file found. Are you sure you ran mission import --file?"
            )
            return

        current_index = mission.get("current_index", 0)

        mission = Mission(file_path, current_index=current_index)
        return mission
    return


async def save_mission(mission):
    print("-- Updating Mission Data...")
    write_to_json(
        {
            mission_file_json: {
                "file_path": mission.file,
                "current_index": mission.current_index,
            }
        }
    )
    return


def wipe_config():
    confirm = True
    while confirm:
        ans = (
            input(f"Are you sure you want to wipe all information in {settings}? y/n: ")
            .lower()
            .strip()
        )
        if ans == "y":
            confirm = False
            break
        if ans == "n":
            return

        print("\nPlease type either y or n to signify yes or no.")

    if ans == "n":
        return

    write_to_json({mission_file_json: {}, drone_instance_json: {"port": None}})
    print("-- Data wiped! This is irreversible.")
