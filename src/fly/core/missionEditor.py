import asyncio
import json
from pathlib import Path

from fly.core.mission import Mission

from fly.utils.geo import haversine_m, Point

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
    return wrapper # runs once for each decorator call during Module Import Time

def _to_f(value, default: float = float("nan")) -> float:
    return default if value is None else float(value)

def _sanitize_waypoint(wp: dict) -> dict:
    # Mirrors Mission.parse_file's None -> NaN coercion so waypoints built at runtime
    # (a CLI prompt) behave identically to ones loaded from a mission file once they reach convert_mission_items_to_plan()
    return {k: (_to_f(v) if v is None else v) for k, v in wp.items()}

class MissionEditor:
    """
    Mid-flight mission editing: pause, modify local json, re-upload, resume

    1. acquire lock to prevent concurrent edits
    2. record current waypoint index
    3. pause the mission and wait for the drone to stop
    4. Load, modify, and save the local json
    5. upload the new mission and resume at corrected index
    """

    def __init__(self, drone, mission: Mission):
        self.drone = drone
        self.mission = mission
        self._lock = asyncio.Lock()

    async def current_index(self) -> int:
        progress = await self.mission.get_mission_progress(self.drone)
        if progress is None:
            return -1
        current, _ = progress
        return current

    async def seconds_until_next_waypoint(self, current: int) -> float: # used by is_safe_to_edit
        # returns estimated seconds until the next waypoint
        # straight-line distance / ground speed. returns inf if speed = 0
        # The GUI uses this to enable the edit button when the value exceeds 8s
        # 1s to detect safe window, 1s to call pause_mission(), 3s drone to stop, 2s to upload, 1s buffer

        info = await self.mission.get_current_next_waypoint_info(self.drone, current)
        current_waypoint, next_waypoint = info
        if next_waypoint is None: # mission complete
            return float("inf")
        if current_waypoint is None: # drone hasn't departed yet
            return float("inf")

        distance = haversine_m(
            Point(current_waypoint.latitude_deg, current_waypoint.longitude_deg),
            Point(next_waypoint.latitude_deg, next_waypoint.longitude_deg)
        )

        ground_speed = await self.drone.current_ground_speed()

        if ground_speed == 0:
            return float("inf")

        return (distance / ground_speed)

    async def is_safe_to_edit(self, time_buffer_s: int = 8) -> bool:
        current = await self.current_index()

        if current < 0:
            # mission not active so we can't insert/append anything
            return False

        time_to_next = await self.seconds_until_next_waypoint(current)

        if time_to_next > time_buffer_s:
            print(f"SAFE: {time_to_next:.1f}s until next waypoint")
            return True
        else:
            print(f"UNSAFE: {time_to_next:.1f}s until next waypoint")
            return False

    # public -----
    @require_safe_edit_window
    async def append_waypoint(self, wp:dict):
        wp = _sanitize_waypoint(wp)
        async with self._lock:
            idx = await self.current_index()
            if not await self.is_safe_to_edit(): # double check: repeated across insert and remove waypoint as well
                print("Safety window closed by the time lock was acquired")
                return

            await self._pause()
            wps = self.mission.waypoints
            wps.append(wp)
            self._save(wps)
            await self._upload_and_resume(wps, idx) # append never changes idx

    @require_safe_edit_window
    async def insert_waypoint(self, at: int, wp: dict):
        wp = _sanitize_waypoint(wp)
        async with self._lock:
            idx = await self.current_index()
            if not await self.is_safe_to_edit():
                print("Safety window closed by the time lock was acquired")
                return

            wps = self.mission.waypoints
            if at < 0 or at > len(wps):
                clamped = max(0, min(at, len(wps)))
                print(f"insert_waypoint: index {at} out of range (0-{len(wps)}), clamping")
                at = clamped

            await self._pause()
            wps.insert(at, wp)
            self._save(wps)
            adjusted_idx = idx + 1 if at <= idx else idx
            await self._upload_and_resume(wps, adjusted_idx)

    @require_safe_edit_window
    async def remove_waypoint(self, at: int):
        async with self._lock:
            idx = await self.current_index()
            if not await self.is_safe_to_edit():
                print("Safety window closed by the time lock was acquired")
                return

            wps = self.mission.waypoints
            if at < 0 or at >= len(wps):
                print(f"remove_waypoint: index {at} out of range (0-{len(wps)-1})") # check happens before pause
                return

            await self._pause()
            del wps[at]
            self._save(wps)
            if at < idx: # remove past waypoint
                adjusted_idx = idx - 1
            elif at == idx: # waypoint being removed is the one the drone is currently flying toward
                adjusted_idx = min(idx, len(wps) - 1)
                # essentially: if it's the final waypoint being removed, the current waypoint
                # should be the past one (go back to the one before final): len(wps) - 1
                # if the waypoint being removed is in the middle of the session, then it should skip over to the next one (idx)
                # if they're the same index number (waypoint being removed is right before final) then they end up on the same result
            else: # remove future waypoint; no change
                adjusted_idx = idx
            await self._upload_and_resume(wps, adjusted_idx)

    # private helpers
    async def _pause (self):
        # pauses the mission and waits for velocity <.1 m/s. 10s timeout
        await self.mission.pause_mission(self.drone)
        try:
            async with asyncio.timeout(10):
                await self.drone.wait_until_stopped(0.1)
        except TimeoutError:
            print("Timeout 10s")

    async def _upload_and_resume(self, waypoints: list[dict], resume_index:int):
        # converts waypoints to missionitems, uploads, sets item, starts mission
        # explicitly sync self.mission.waypoints to the edited list so convert_mission_items_to_plan always uses the correct data
        self.mission.waypoints = waypoints

        try:
            await self.mission.upload_mission(self.drone)
            await self.mission.set_current_mission_target(self.drone, resume_index)
            await self.mission.start_mission(self.drone)
        except Exception as e:
            print(f"_upload_and_resume failed at resume_index={resume_index}: {e}")
            raise

    def _save(self, waypoints: list[dict]):
        # writes waypoints
        # there will no longer be a _meta key planned
        if self.mission.path is None:
            print("Cannot save: mission has no file path")
            return
        with open(self.mission.path, "w") as f:
            json.dump([self.mission.RTL] + waypoints, f, indent = 2)
