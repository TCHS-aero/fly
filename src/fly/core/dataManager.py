import json
from pathlib import Path

import fly
PACKAGE_DIR = Path(fly.__file__).resolve().parent # tracks the top of the fly package
settings = PACKAGE_DIR / "config" / "settings.json"

def pull_data():
    try:
        with open(settings, "r", encoding="utf-8") as read_file:
            data = json.load(read_file)
            return data
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"-- Error reading data from {settings}. Returning empty data.")
        print(e)
        return None

def update_port_data(port: str | None = None, history: list | None = None):
    try:
        existing_data = pull_data() or {}

        if port is not None:
            existing_data["port"] = port
        if history is not None:
            existing_data["port-history"] = history

        with open(settings, "w", encoding="utf-8") as write_file:
            json.dump(existing_data, write_file, ensure_ascii=False, indent=4)
        print("-- Writing Success!")
    except Exception as e:
        print(e)

def update_mission_data(current: int | None = None, total: int | None = None):
    try:
        existing_data = pull_data() or {}

        if current is not None:
            existing_data["current-mission-progress"] = current
        if total is not None:
            existing_data["total-mission-progress"] = total

        with open(settings, "w", encoding="utf-8") as write_file:
            json.dump(existing_data, write_file, ensure_ascii=False, indent=4)
        print("-- Writing Success!")
    except Exception as e:
        print(e)

def update_setting(key: str, value) -> None:
    # Generic single-key writer, for settings that don't warrant their own update_*_data() function
    # (rtsp-url, model-path, confidence-threshold).
    try:
        existing_data = pull_data() or {}
        if not existing_data:
            print("-- Info: existing data is empty, writing into new file.")
        existing_data[key] = value
        with open(settings, "w", encoding="utf-8") as write_file:
            json.dump(existing_data, write_file, ensure_ascii=False, indent = 4)
        print("-- Writing Success!")
    except Exception as e:
        print(e)

def get_setting(key: str, default=None):
    data = pull_data()
    if data is None:
        return default
    return data.get(key, default)
