""" Not in use since it may slow down the mission too much

import asyncio
from fly.utils.geo import offset_coords, haversine_m, bearing_between, pixel_to_ground, Point

def approach_waypoints(poi_pos: Point, approach_alt_m: float = 15.0, delivery_alt_m: float = 5.0) -> list[dict]:
    # returns 1 waypoint for a POI approach, in mission_waypoints.json format:
        # poi_pos, approach_alt_m
    # is_fly_through=False
    # Can be passed directly to MissionEditor

def centering_nudge(
    detected_xy: tuple[int,int],
    image_wh: tuple[int,int],
    drone_pos: Point,
    alt_m: float,
    fov_h_deg: float,
    fov_v_deg: float,
    heading_deg: float,
) -> tuple[float, float]:
    # Returns (north_m, east_m) the drone must move to place the detection directly below it
    # Delegates projection to pixel_to_ground(), then subtracts drone_pos to get the offset
    # Returns (0.0, 0.0) if the detection is already within 10% of the image width of center

async def run_centering_loop(
    drone,
    capture_fn,
    detect_fn,
    max_steps: int = 5,
    threshold_px: int = 30,
    fov_h_deg: float = 90.0,
    fov_v_deg: float = 67.5,
) -> Point:
    # iteratively centers drone over the target.
    # Each step:
        # 1. capture_fn() -> (payload, image_path)
        # 2. detect_fn(image_path) -> detections
        # 3. Best detection within threshold_px of image center -> done, return payload.pos
        # 4. Else centering_nudge() -> drone.move_to_location(nudged coords)
    # returns payload.pos when centered, or best position reached after max steps
"""
