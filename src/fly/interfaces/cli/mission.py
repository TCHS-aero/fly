# Mission and MissionEditor
import json

import asyncclick as click

from fly.core.mission import Mission
from fly.core.missionEditor import MissionEditor
from fly.interfaces.cli.session import port_option, require_drone, require_mission


@click.group(help = "Load, upload, start, pause, and inspect missions.")
def mission():
    pass

@mission.command(name = "load", help = "Parse and validate a mission file (no drone needed).")
@click.option("--file", "file_", required=True, type=click.Path(exists=True), help="Mission JSON file.")
def mission_load(file_):
    m = require_mission(file_)
    print(f"-- {file_}: {m.total_waypoints} waypoint(s), RTL-after-mission={m.RTL}")
    for i, wp in enumerate(m.waypoints):
        print(
            f"  [{i}] lat={wp['latitude_deg']:.6f} lon={wp['longitude_deg']:.6f} "
            f"alt={wp['relative_altitude_m']}m speed={wp['speed_m_s']}m/s"
        )

@mission.command(name="upload", help="Upload a mission file to the connected drone.")
@click.option("--file", "file_", required=True, type=click.Path(exists=True), help="Mission JSON file.")
@port_option
@click.option("--start", is_flag=True, default = False, help="Start the mission immediately after uploading.")
async def mission_upload(file_, port, start):
    m = require_mission(file_)
    drone = await require_drone(port)

    print(f"-- Uploading {m.total_waypoints} waypoint(s)...")
    await m.upload_mission(drone)
    print("-- Upload complete.")

    if start:
        await m.start_mission(drone)
        print("-- Mission started.")

@mission.command(name="start", help="Start (or resume, if paused) the mission on the connected drone.")
@port_option
async def mission_start(port):
    drone = await require_drone(port)
    m = Mission()
    await m.start_mission(drone)
    print("-- Mission started.")

@mission.command(name="pause", help="Pause the mission currently running on the connect drone.")
@port_option
async def mission_pause(port):
    drone = await require_drone(port)
    m = Mission()
    await m.pause_mission(drone)
    print("-- Mission paused.")

@mission.command(name="status", help="Show mission progress on the connected drone.")
@port_option
async def mission_status(port):
    drone = await require_drone(port)
    m = Mission()

    progress = await m.get_mission_progress(drone)
    finished = await m.is_mission_finished(drone)
    rtl = await m.get_return_to_launch_after_mission(drone)
    if progress:
        current, total = progress
        print(f"-- waypoint {current}/{total}   finished={finished}   RTL-after-mission={rtl}")
    else:
        print(f"-- no progress reported (no mission active?)   finished={finished}   RTL-after-mission={rtl}")

@mission.command(name="download", help="Download and print the mission currently stored on the drone.")
@port_option
async def mission_download(port):
    drone = await require_drone(port)
    m = Mission()
    plan = await m.download_mission(drone)
    if not plan:
        print("-- Could not download mission.")
        raise SystemExit(1)
    items = list(plan.mission_items)
    print(f"-- {len(items)} waypoint(s) stored on the drone:")
    for i, item in enumerate(items):
        print(f"   [{i}] lat={item.latitude_deg:.6f} lon={item.longitude_deg:.6f} alt={item.relative_altitude_m}m")

# mid-flight editing (core.missionEditor) ----------------------------

@click.group(name="edit", help = "Mid-flight mission editing (append/insert/remove) via MissionEditor.")
def mission_edit():
    pass

mission.add_command(mission_edit)

def waypoint_options(fn):
    # shared flag set for append/insert -- mirrors the mission_waypoints.json schema
    # see Mission.convert_mission_items_to_plan for the field list
    fn = click.option("--lat",   type=float,     default=None,  help="Latitude, degrees.")(fn)
    fn = click.option("--lon",   type=float,     default=None,  help="Longitude, degrees.")(fn)
    fn = click.option("--alt",   type=float,     default=None,  help="Relative altitude, meters.")(fn)
    fn = click.option("--speed", type=float,     default=10.0,   show_default=True, help="Speed, m/s")(fn)
    fn = click.option("--acceptance-radius",     type=float,    default=10.0, show_default=True, help="Acceptance radius, meters.")(fn)
    fn = click.option("--yaw",   type=float,     default=0.0,   show_default=True, help="Yaw, degrees.")(fn)
    fn = click.option("--fly-through/--stop-at", default=True,  show_default=True, help="Fly through vs stabilize at the waypoint.")(fn)
    fn = click.option("--loiter-s", type=float,  default=0.0,   show_default=True, help="Loiter time, seconds.")(fn)
    fn = click.option("--camera-action", type=int, default=0,   show_default=True,
        help="MissionItem.CameraAction int (0=NONE, 1=TAKE_PHOTO, 4=START_VIDEO, 5=STOP_VIDEO, ...)",
    )(fn)
    fn = click.option("--vehicle-action", type=int, default=0,  show_default=True,
        help="MissionItem.VehicleAction int (0=NONE, 1=TAKEOFF, 2=LAND, ...)"
    )(fn)
    return fn

def waypoint_from_kwargs(kwargs) -> dict:
    return {
        "latitude_deg": kwargs['lat'],
        "longitude_deg": kwargs['lon'],
        "relative_altitude_m": kwargs['alt'],
        "speed_m_s": kwargs['speed'],
        "is_fly_through": kwargs['fly_through'],
        "gimbal_pitch_deg": None,
        "gimbal_yaw_deg": None,
        "camera_action": kwargs['camera_action'],
        "loiter_time_s": kwargs['loiter_s'],
        "camera_photo_interval_s": 0.0,
        "acceptance_radius_m": kwargs['acceptance_radius'],
        "yaw_deg": kwargs['yaw'],
        "camera_photo_distance_m": 0.0,
        "vehicle_action": kwargs['vehicle_action']
    }

async def _load_editor(file_: str, port: str | None) -> MissionEditor:
    # shared by append/insert/remove
    m = require_mission(file_)
    drone = await require_drone(port)
    return MissionEditor(drone, m)

def _extract_waypoints(waypoints_file: str | None, kwargs: dict) -> list[dict]:
    # validate flags/options and return a list of waypoint dicts to append or insert
    if waypoints_file:
        with open(waypoints_file) as f:
            data = json.load(f)
        if not isinstance(data, list) or len(data) < 2:
            raise click.BadParameter(f"{waypoints_file} is not a valid mission file (expected [RTL, wp, ...]).")
        return data [1:]

    lat, lon, alt = kwargs.get("lat"), kwargs.get("lon"), kwargs.get("alt")
    if lat is None or lon is None or alt is None:
        raise click.UsageError("Specify either --waypoints-file or all of --lat, --lon, and --alt.")
    return [waypoint_from_kwargs(kwargs)]

@mission_edit.command(name="append", help="Append a waypoint to the end of the active mission.")
@click.option("--file", "file_", required=True, type=click.Path(exists=True), help="The mission file currently active on the drone (edits saved in this GCS)")
@port_option
@click.option("--waypoints-file", type=click.Path(exists=True), default=None, help="A mission JSON file whose waypoints will be appended (RTL flag at index 0 is skipped). Mutually exclusive with manual waypoint flags.")
@waypoint_options
async def edit_append(file_, port, waypoints_file, **kwargs):
    wps = _extract_waypoints(waypoints_file, kwargs)
    editor = await _load_editor(file_, port)
    for wp in wps:
        await editor.append_waypoint(wp)

    print(f"-- Append was requested: {len(wps)} waypoint(s).")

@mission_edit.command(name="insert", help="Insert a waypoint at a specific index in the active mission.")
@click.option("--file", "file_", required=True, type=click.Path(exists=True), help="The mission file currently active on the drone (edits saved in this GCS)")
@port_option
@click.option("--at", type=int, required=True, help="Index to insert at (0 = before the first waypoint).")
@click.option("--waypoints-file", type=click.Path(exists=True), default=None, help="A mission JSON file whose waypoints will be appended (RTL flag at index 0 is skipped). Mutually exclusive with manual waypoint flags.")
@waypoint_options
async def edit_insert(file_, port, at, waypoints_file, **kwargs):
    wps = _extract_waypoints(waypoints_file, kwargs)
    editor = await _load_editor(file_, port)
    for i, wp in enumerate(wps):
        await editor.insert_waypoint(at + i, wp)

    print(f"-- Insert was requested: {len(wps)} waypoint(s) starting at index {at}.")

@mission_edit.command(name="remove", help="Remove a waypoint at a specific index from the active mission.")
@click.option("--file", "file_", required=True, type=click.Path(exists=True), help="The mission file currently active on the drone (edits saved in this GCS)")
@port_option
@click.option("--at", type=int, required=True, help="Index to insert at (0 = before the first waypoint).")
async def edit_remove(file_, port, at):
    editor = await _load_editor(file_, port)
    await editor.remove_waypoint(at)
    print("-- Remove was requested.")
