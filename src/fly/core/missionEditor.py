import asyncio
import json
import math
from pathlib import Path
from time import time

from fly.core.mission import Mission
from fly.core.dataManager import pull_data

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

    def __init__(self, drone, mission):
        self.drone = drone
        self.mission = mission
        self._lock = asyncio.Lock()

    def haversine(lat1, lon1, lat2, lon2):
        R = 6371000

        lat1, lon1, lat2, lon2 = map(
            math.radians,
            [lat1, lon1, lat2, lon2]
        )

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(lat1)
            * math.cos(lat2)
            * math.sin(dlon / 2) ** 2
        )

        return 2 * R * math.asin(math.sqrt(a))

    async def current_total_index(self) -> int:
        data = pull_data()

        current, total = (
            data["current-mission-progress"], 
            data["total-mission-progress"]
        )

        return current, total

    async def seconds_until_next_waypoint(self) -> float:
        # returns estimated seconds until the next waypoint
        # straight-line distance / ground speed. returns inf if speed = 0
        # The GUI uses this to enable the edit button when the value exceeds 8s
        # 1s to detect safe window, 1s to call pause_mission(), 3s drone to stop, 2s to upload, 1s buffer

        current, total = await self.current_total_index()
        current_waypoint, next_waypoint = self.mission.get_current_next_waypoint_information(self.drone, current)
        
        distance = self.haversine(
            current_waypoint["lat"],
            current_waypoint["lon"],
            next_waypoint["lat"],
            next_waypoint["lon"]
        )

        ground_speed = await self.drone.current_ground_speed()
        
        if speed == 0:
            return float("inf")

        return (distance / ground_speed)

    async def is_safe_to_edit(self, time_buffer_s: int = 8) -> bool:
        current, total = await self.current_total_index()

        if current < 0:
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
        async with self._lock:
            idx = (await self.current_total_index())[0]
            await self._pause()
            wps = self.mission.waypoints
            wps.append(wp)
            clean = self.save_waypoints(wps)
            await self._upload_and_resume(clean, idx)

    @require_safe_edit_window
    async def insert_waypoint(self, at: int, wp: dict):
        async with self._lock:
            idx = (await self.current_total_index())[0]
            await self._pause()
            wps = self.mission.waypoints
            wps.insert(0, wp)
            clean = self.save_waypoints(wps)
            await self._upload_and_resume(clean, idx)

    @require_safe_edit_window
    async def remove_waypoint(self, at: int):
        async with self._lock:
            idx = (await self.current_total_index())[0]
            await self._pause()
            wps = self.mission.waypoints
            wps.pop(at)
            clean = self.save_waypoints(wps)
            await self._upload_and_resume(clean, idx)

    # private helpers
    async def _pause (self):
        # pauses the mission and waits for velocity <.1 m/s. 10s timeout

        try:
            async with asyncio.timeout(10):    
                await self.mission.pause_mission()
                await self.drone.wait_until_stopped(0.1)
        except TimeoutError:
            print("Timeout 10s")
            return

    async def _upload_and_resume(self, waypoints: list[dict], resume_index:int):
        # converts waypoints to missionitems, uploads, sets item, starts mission
        await self.mission.upload_mission(self.drone)
        await self.mission.set_current_mission_target(self.drone, resume_index)
        await self.start_mission(self.drone)

    def save_waypoints(self, waypoints: list[dict]):
        # writes waypoints, stripping '_meta' keys MAVSDK would reject
        with open(self.path, "w") as f:
            json.dump(waypoints, f, indent = 2)

        clean = [
            waypoints[0], # retain the rtl flag
            *({k: v for k, v in wp.items() if k!="_meta"} for wp in waypoints[1:])
        ]

        return clean

        # _meta is for something like this:
            # "_meta": {"type": "poi_approach", "poi_id":1}
        # It will be appended in mission_waypoints.json, and then
        # removed before being sent to drone
