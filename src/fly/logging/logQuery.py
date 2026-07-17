# Not called by any other module; poiManager's _nearest may seem similar to entries_near but it only queries POIs
# Helpful for testing if used in cli

from fly.logging.flightLog import FlightLog
from fly.utils.geo import haversine_m, Point

def entries_near(log: FlightLog, pos:Point, radius_m: float) -> list[dict]:
    # All entries whose capture position is within radius_m of pos,
    # sorted by ascending distance
    hits: list[tuple[float, dict]] = []
    for entry in log.all_entries():
        d = haversine_m(pos, Point(entry["lat"], entry["lon"]))
        if  d<=radius_m:
            hits.append((d, entry))
    return [e for _, e in sorted(hits, key=lambda x: x[0])]

def entries_by_phase(log: FlightLog, phase: str) -> list[dict]:
    # all entries matching phase ("survey" | "calibration" | "manual")
    return [e for e in log.all_entries() if e.get("phase") == phase]

def entries_by_waypoint(log: FlightLog, wp_index: int) -> list[dict]:
    # all entries within the ISO 8601 timestamp range [ts_start, ts_end], inclusive
    return [e for e in log.all_entries() if e.get("wp_index") == wp_index]

def entries_in_window(log: FlightLog, ts_start: str,  ts_end:str) -> list[dict]:
    return[
        e for e in log.all_entries()
        if ts_start <= e.get("ts", "") <= ts_end
    ]

def nearest_entry(log: FlightLog, pos: Point) -> dict | None:
    # the entry whose capture position is closest to pos. None if log is empty
    entries = log.all_entries()
    if not entries:
        return None
    return min(entries, key=lambda e: haversine_m(pos, Point(e["lat"], e["lon"])))
