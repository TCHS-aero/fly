# Mission and MissionEditor

import asyncclick as click

from fly.core.mission import Mission
from fly.core.missionEditor import MissionEditor
from fly.interfaces.cli.session import get_connected_drone, load_mission

@click.group(help = "Load, upload, start, pause, and inspect missions.")
def mission():
    pass

@mission.command(name = "load", help = "Parse and validate a mission file (no drone needed).")
@click.option("--file", "file_", required=True, type=click.Path(exists=True), help="Mission JSON file.")
def mission_load(file_):
    m = load_mission(file_)
    if not m:
        raise SystemExit(1)
    print(f"-- {file_}: {m.total_waypoints} waypoint(s), RTL-after-mission={m.RTL}")
    for i, wp in enumerate(m.waypoints):
        print(
            f"  [{i}] lat={wp['latitude_deg']:.6f} lon={wp['longitude_deg']:.6f} "
            f"alt={wp['relative_altitude_m']}m speed={wp['speed_m_s']}m/s"
        )

@mission.command(name="upload", help="Upload a mission file to the connected drone.")
@click.option("--file", "file_", required=True, type=click.Path(exists=True), help="Mission JSON file.")
@click.option("--port", help="Connection port. Defaults to the last-used port.")
@click.option("--start", is_flag=True, default = False, help="Start the mission immediately after uploading.")
async def mission_upload(file_, port, start):
    m = load_mission(file_)
    if not m:
        raise SystemExit(1)

    drone = await get_connected_drone(port)
    if not drone:
        raise SystemExit(1)

    print(f"-- Uploading {m.total_waypoints} waypoint(s)...")
    await m.upload_mission(drone)
    print("-- Upload complete.")

    if start:
        await m.start_mission(drone)
        print("-- Mission started.")

@mission.command(name="start", help="Start (or resume, if paused) the mission on the connected drone.")
@click.option("--port", help="Connection port. Defaults to the last-used port.")
async def mission_start(port):
    drone = await get_connected_drone(port)
    if not drone:
        raise SystemExit(1)
    m = Mission()
    await m.start_mission(drone)
    print("-- Mission started.")

@mission.command(name="pause", help="Pause the mission currently running on the connect drone.")
@click.option("--port", help="Connection port. Defaults to the last-used port.")
async def mission_pause(port):
    drone = await get_connected_drone(port)
    if not drone:
        raise SystemExit(1)
    m = Mission()
    await m.pause_mission(drone)
    print("-- Mission paused.")

@mission.command(name="status", help="Show mission progress on the connected drone.")
@click.option("--port", help="Connection port. Defaults to the last-used port.")
async def mission_status(port):
    drone = await get_connected_drone(port)
    if not drone:
        raise SystemExit(1)
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
@click.option("--port", help="Connection port. Defaults to the last-used port.")
async def mission_download(port):
    drone = await get_connected_drone(port)
    if not drone:
        raise SystemExit(1)
    m = Mission()
    plan = await m.download_mission(drone)
    if not plan:
        raise SystemExit(1)
    items = list(plan.mission_items)
    print(f"-- {len(items)} waypoint(s) stored on the drone:")
    for i, item in enumerate(items):
        print(f"   [{i}] lat={item.latitude_deg:.6f} lon={item.longitude_deg:.6f} alt={item.relative_altitude_m}m")

# mid-flight editing (new pipeline: core.missionEditor)
