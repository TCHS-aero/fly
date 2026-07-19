from typing import NamedTuple

import numpy as np
import pymap3d as pm
from geographiclib.geodesic import Geodesic

# to find great-circle distance in meters between two WGS84 points
from haversine import Direction, Unit, haversine, inverse_haversine


class Point(NamedTuple):
    lat: float
    lon: float

def haversine_m(pt1: Point, pt2: Point) -> float:
    return haversine(
        pt1, pt2, unit=Unit.METERS
    )  # could also use Geodesic but haversine is more lightweight and accurate enough


# choosing to input North and East offset since it is native to MAVSDK (uses NED), rather than bearing
def offset_coords(start_pt: Point, north_m: float, east_m: float) -> Point:
    # returns (lat, lon) after moving a certain meters north and east
    temp_pt = inverse_haversine(start_pt, north_m, Direction.NORTH, unit=Unit.METERS)
    res = inverse_haversine(temp_pt, east_m, Direction.EAST, unit=Unit.METERS)
    return Point(lat=res[0], lon=res[1])

def bearing_between(p1: Point, p2: Point) -> float:
    # Compass bearing from p1 to p2
    geo = Geodesic.WGS84.Inverse(p1.lat, p1.lon, p2.lat, p2.lon)

    bearing = geo[
        "azi1"
    ]  # s12: distance, azi1: bearing from starting, azi2: bearing from endpoint
    return bearing % 360


def pixel_to_ground(
    detected_xy: tuple[int, int],  # comes from the ai output
    image_wh: tuple[int, int],
    drone_pos: Point,
    alt_m: float,
    heading_deg: float,
    fov_h_deg: float = 81,
    fov_v_deg: float = 51.3,
) -> Point:
    # projects a pixel detection to ground coordinate, assuming nadir camera (90deg)
    cx, cy = detected_xy
    w, h = image_wh

    # camera intrinsic matrix
    fx = (w / 2.0) / np.tan(
        np.radians(fov_h_deg / 2.0)
    )  # focal lengths expressed in pixels rather than mm
    fy = (h / 2.0) / np.tan(np.radians(fov_v_deg / 2.0))
    K = np.array([[fx, 0, w / 2.0], [0, fy, h / 2.0], [0, 0, 1.0]])

    # back-project pixel -> camera-frame ray, scale to ground plane
    ray = np.linalg.solve(K, np.array([cx, cy, 1.0]))
    ray *= alt_m / ray[2]
    dx_m = ray[0]  # meters in image +x direction (right)
    dy_m = ray[1]  # meters in image +y (aft)

    # rotate image frame -> NED via drone heaing
    hr = np.radians(heading_deg)
    R = np.array([[-np.sin(hr), -np.cos(hr)], [np.cos(hr), -np.sin(hr)]])
    north_m, east_m = R @ np.array([dx_m, dy_m])

    # NED offset -> WGS-84 via pymap3d
    # pm.ned2geodetic handles the ellipsoidal maths
    (
        lat,
        lon,
        _alt,
    ) = pm.ned2geodetic( # (n, e, d, lat0, lon0, h0)
        north_m, east_m, float(alt_m), drone_pos.lat, drone_pos.lon, 0.0
    )
    return Point(lat=lat, lon=lon)
