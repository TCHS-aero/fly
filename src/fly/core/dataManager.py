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

def _write_data(data: dict) -> None:
    # shared read-modify-write tail for update_port_data/update_mission_data/update_setting
    try:
        with open(settings, "w", encoding="utf-8") as write_file:
            json.dump(data, write_file, ensure_ascii=False, indent=4)
        print("-- Writing Success!")
    except Exception as e:  # noqa
        print(e)

def update_port_data(port: str | None = None, history: list | None = None):
    data = pull_data() or {}
    if port is not None:
        data["port"] = port
    if history is not None:
        data["port-history"] = history
    _write_data(data)


def update_mission_data(current: int | None = None, total: int | None = None):
    data = pull_data() or {}
    if current is not None:
        data["current-mission-progress"] = current
    if total is not None:
        data["total-mission-progress"] = total
    _write_data(data)

def update_setting(key: str, value) -> None:
    # Generic single-key writer, for settings that don't warrant their own update_*_data() function
    # (rtsp-url, model-path, confidence-threshold).
    data = pull_data() or {}
    if not data:
        print("-- Info: existing data is empty, writing into new file.")
    data[key] = value
    _write_data(data)

def get_setting(key: str, default=None):
    data = pull_data()
    if data is None:
        return default
    return data.get(key, default)
