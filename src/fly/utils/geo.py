import math
# to find great-circle distance in meters between two WGS84 points
from haversine import haversine, Unit, inverse_haversine, Direction
from geographiclib.geodesic import Geodesic

Point = tuple[float,float] # lat_deg, lon_deg

def haversine_m(pt1: Point, pt2: Point) -> float:
    return haversine(pt1, pt2, unit = Unit.METERS) # could also use Geodesic but haversine is more lightweight and accurate enough

# choosing to input North and East offset since it is native to MAVSDK (uses NED), rather than bearing
def offset_coords(start_pt: Point, north_m: float, east_m: float) -> Point:
    # returns (lat, lon) after moving a certain meters north and east
    temp_pt = inverse_haversine(start_pt, north_m, Direction.NORTH, unit=Unit.METERS)
    return inverse_haversine(temp_pt, east_m, Direction.EAST, unit=Unit.METERS)

def bearing_between(p1: Point, p2: Point) -> float:
    # Compass bearing from p1 to p2
    geo = Geodesic.WGS84.Inverse(p1[0], p1[1], p2[0], p2[1])

    bearing = geo['azil'] # s12: distance, azi1: bearing from starting, azi2: bearing from endpoint
    return bearing % 360

def pixel_to_ground(
    detected_xy: tuple[int,int],
    image_wh: tuple[int,int],
    drone_pt: Point,
    alt_m: float,
    fov_h_deg: float,
    fov_v_deg: float,
    heading_deg: float
) -> Point:
    # projects a pixel detection to ground coordinate, assuming nadir camera (90deg)
