from fly.logging.flightLog import FlightLog
from fly.utils.geo import haversine_m, Point

def entries_near(log: FlightLog, pos:Point, radius_m: float) -> list[dict]:
    # All entries whose capture position is within radius_m of pos,
    # sorted by ascending distance

def entries_by_phase(log: FlightLog, phase: str) -> list[dict]:
    # all entries matching phase ("survey" | "calibration" | "manual")

def entries_by_waypoint(log: FlightLog, wp_index: int) -> list[dict]:
    # all entries within the ISO 8601 timestamp range [ts_start, ts_end], inclusive

def nearest_entry(log: FlightLog, pos: Point) -> dict | None:
    # the entry whose capture position is closest to pos. None of log is empty
