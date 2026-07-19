import asyncio
import json
import os
import aiofiles
from pathlib import Path

from fly.utils.geo import Point, haversine_m


class POIManager:
    # Detectio aggregation, deduplication, and lifecycle management.
    # Detections within DEDUP_RADIUS_M of an existing POI are merged using a running weighted average.
    DEDUP_RADIUS_M = 5.0

    def __init__(self, registry_path: str = "poi_registry.json"):
        self.path = Path(registry_path)
        self._pois: dict[int, dict] = {}
        self._next_id = 1
        self._lock = asyncio.Lock()
        self._subscribers: list = []

    async def load(self):
        # loads poi_registry.json. Reconstructs _pois and _next_id
        try:
            with open(self.path) as f:
                data = json.load(f)
            self._pois = {p["poi_id"]: p for p in data}
            self._next_id = max(self._pois.keys(), default=0) + 1
        except (FileNotFoundError, json.JSONDecodeError):
            self._pois = {}
            self._next_id = 1

    async def save(self):
        # atomically writes all POIS ( write temp file, then rename)
        tmp = self.path.with_suffix(".poi_tmp")
        payload = json.dumps(list(self._pois.values()), indent = 2)
        async with aiofiles.open(tmp, "w") as f:
            await f.write(payload)
        os.replace(str(tmp), str(self.path))

    async def add_detection(self, pos: Point, confidence: float, source_image:str) -> tuple[int, bool]:
        # merges into nearset POI within DEDUP_RADIUS_M, or creates a new one
        # returns (poi_id, is_new). Saves and notifies every call
        # Running weighted average merge:
            # merged_lat = (lat * n + pos.lat) / (n+1)
            # merged_lon = (lon * n + pos.lon) / (n+1)
            # merged_conf = (conf * n + new_conf) / (n+1)

        async with self._lock:
            poi_id = self._nearest(pos)
            if poi_id is not None:
                p = self._pois[poi_id]
                n = p["detection_count"]
                p["lat"] = (p["lat"] * n + pos.lat) / (n+1)
                p["lon"] = (p["lon"] * n + pos.lon) / (n+1)
                p["confidence_avg"] = (p["confidence_avg"] * n + confidence) / (n+1)
                p["detection_count"] += 1
                p["source_images"].append(source_image)
                is_new = False
            else:
                poi_id = self._next_id
                self._next_id += 1
                self._pois[poi_id] = {
                    "poi_id": poi_id, "status": "candidate",
                    "lat": pos.lat, "lon": pos.lon,
                    "confidence_avg": confidence, "detection_count": 1,
                    "source_images": [source_image],
                }
                is_new=True
            await self.save()
            self._notify(poi_id, "created" if is_new else "updated")
            return poi_id, is_new

    def get(self, poi_id: int) -> dict | None:
        return self._pois.get(poi_id)

    def get_all(self, status: str | None = None) -> list[dict]:
        pois = list(self._pois.values())
        return [p for p in pois if p["status"] == status] if status else pois

    async def update_status(self, poi_id: int, new_status: str):
        # valid transitions: candidate -> queued -> approached -> delivered | dismissed
        if poi_id not in self._pois:
            raise KeyError(f"No POI with id {poi_id}")
        self._pois[poi_id]["status"] = new_status
        await self.save()
        self._notify(poi_id, "status_changed")

    def subscribe(self, callback):
        # callback(poi_id: int, event: str, poi:dict) on every change
        self._subscribers.append(callback)

    # private

    def _nearest(self, pos:Point) -> int | None:
        best, best_d = None, self.DEDUP_RADIUS_M
        for pid, p in self._pois.items():
            d = haversine_m(pos, Point(p["lat"], p["lon"]))
            if d<best_d:
                best, best_d = pid, d
        return best

    def _notify(self, poi_id: int, event: str):
        for cb in self._subscribers:
            cb(poi_id, event, self._pois.get(poi_id))
