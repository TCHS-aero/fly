from dataclasses import dataclass
from datetime import datetime, timezone


@dataclass  # dataclass writes the __init__ automatically
class ImagePayload:
    # One captured frame plus drone state at that moment
    ts: str  # ISO 8601 UTC
    lat: float
    lon: float
    alt_rel: float  # meters above home
    wp_index: int
    phase: str  # survey | calibration | manual
    filename: str

    @property
    def pos(self) -> Point:
        # Bridge to any geo function that takes a Point
        return (self.lat, self.lon)

    @staticmethod
    def now_ts() -> str:  # `:-3` chops off 3 digits to get milliseconds
        return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
