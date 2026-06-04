import asyncio
import json
from pathlib import Path
from time import time

from fly.core.mission import Mission

import functools
def require_safe_edit_window(func):
    # checks if it's safe to edit. decorator so code isn't duplicated
    @functools.wraps(func)
    async def wrapper(self, *args, **kwargs):
        if not await self.is_safe_to_edit():
            print("Cannot edit mission now; not enough time before next waypoint")
            return
        # if safe, do original function
        return await func(self, *args, **kwargs)
    return wrapper # afaik this runs once for each decorator call during Module Import Time

class MissionEditor:
    """
    Mid-flight mission editing: pause, modify local json, re-upload, resume

    1. acquire lock to prevent concurrent edits
    2. record current waypoint index
    3. pause the mission and wait for the drone to stop
    4. Load, modify, and save the local json
    5. upload the new mission and resume at corrected index
    """

    def __init__(self, drone, local_mission_path: str):
        self.drone = drone
        self.path = Path(local_mission_path)
        self._lock = asyncio.Lock()

    async def current_index(self) -> int:
        # returns current waypoint index from mavsdk, or -1 if not active
        pass

    async def seconds_until_next_waypoint(self) -> float:
        # returns estimated seconds until the next waypoint
        # straight-line distance / ground speed. returns inf if speed = 0
        # The GUI uses this to enable the edit button when the value exceeds 8s
        # 1s to detect safe window, 1s to call pause_mission(), 3s drone to stop, 2s to upload, 1s buffer
        pass

    async def is_safe_to_edit(self, time_buffer_s: int = 8) -> bool:
        if await self.current_index() < 0:
            # mission not active so we can't insert/append anything
            return False
        time_to_next = await self.seconds_until_next_waypoint()

        if time_to_next > time_buffer_s:
            print(f"SAFE: {time_to_next:.1f}s until next waypoint")
            return True
        else:
            print(f"UNSAFE: {time_to_next:.1f}s until next waypoint")
            return False

    @require_safe_edit_window
    async def append_waypoint(self, wp:dict):
        # appends waypoint to end of mission
        async with self._lock:
            idx = await self.current_index()
            await self._pause()
            wps = self.load_waypoints()
            wps.append(wp)
            self.save_waypoints(wps)
            await self._upload_and_resume(wps,idx)

    @require_safe_edit_window
    async def insert_waypoint(self, at: int, wp: dict):
        # insert a waypoint at position 'at'
        pass

    @require_safe_edit_window
    async def remove_waypoint(self, at: int):
        pass

    # private helpers

    async def _pause (self):
        # pauses the mission and waits for velocity <.1 m/s. 10s timeout

    async def _upload_and_resume(self, waypoints: list[dict], resume_index:int):
        # converts waypoints to missionitems, uploads, sets item, starts mission
        pass

    # JSON I/O
    def load_waypoints(self) -> list[dict]:
        with open(self.path) as f:
            return json.load(f)

    def save_waypoints(self, waypoints: list[dict]):
        # writes waypoints, stripping '_meta' keys MAVSDK would reject
        clean = [{k: v for k, v in wp.items() if k!="_meta"} for wp in waypoints]
        with open(self.path, "w") as f:
            json.dump(clean, f, indent = 2)

        # _meta is for something like this:
            # "_meta": {"type": "poi_approach", "poi_id":1}
        # It will be appended in mission_waypoints.json, and then
        # removed before being sent to drone
