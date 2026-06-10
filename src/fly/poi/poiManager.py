import asyncio
import json
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
        self._suscribers: list = []
