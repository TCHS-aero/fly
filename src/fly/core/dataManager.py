import json

from importlib.resources import files
from fly.core.drone import Drone
from fly.core.mission import Mission

settings = files("fly.config").joinpath("settings.json")

def pull_port_data():
    try:
        with open(settings, "r") as read_file:
            data = json.load(read_file)
            return (data["port"], data["port-history"])
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"-- Error reading data from {settings}. Returning empty data.")
        print(e)
        return None
    
def update_port_data(port: string = None, history: list = None):
    try:
        existing_data = pull_port_data()
        new_port, new_history = existing_data

        if port is not None:
            new_port = port
        if history is not None:
            new_history = history

        final_data = {"port": new_port, "port-history": new_history}

        with open(settings, "w") as write_file:
            print("-- Writing...")
            json.dump(final_data, write_file, ensure_ascii=False, indent=4)
    except Exception as e:
        print(e)

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

    update_port_data(port = "", history = [])
    print("-- Data wiped! This is irreversible.")
