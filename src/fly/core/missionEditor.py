import asyncio
import json
from pathlib import Path

from fly.core.mission import Mission


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

    async def current_index(self) -> init:
        # returns current waypoint index from mavsdk, or -1 if not active

    async def seconds_until_next_waypoint(self) -> float:
        # returns estimated seconds until the next waypoint
        # straight-line distance / ground speed. returns inf if speed = 0
        # The GUI uses this to enable the edit button when the value exceeds 8s

    async def append_waypoint(self, wp:dict):
        # appends waypoint to end of mission
        async with self._lock:
            idx = await self.current_index()
            await self._pause()
            wps = self.load_waypoints()
            wps.append(wp)
            self.save_waypoints(wps)
            await self._upload_and_resume(wps,idx)

    async def insert_waypoint(self, at: int, wp: dict):
        # insert a waypoint at position 'at'

    async def remove_waypoint(self, at: int):
        # removes waypoint at position 'at'

    # private helpers

    async def _pause (self):
        # pauses the mission and waits for velocity <.1 m/s. 10s timeout

    async def _upload_and_resume(self, waypoints: list[dict], resume_index:int):
        # converts waypoints to missionitems, uploads, sets item, starts mission

    # JSON I/O
    def load_waypoints(self) -> list[dict]:
        with open(self.path) as f:
            return json.load(f)

    def save_waypoints(self, waypoints: list[dict]):
        # writes waypoints, stripping '_meta' keys MAVSDK would reject
        clean = [{k: v for k, v in wp.items() if k!="_meta"} for wp in waypoints]
        with open(self.path, "w") as f:
            json.dump(clean, f, indent = 2)
