# wires everything together via FlightSession
# ctrl + c doesn't do anything to the drone - the drone will still have a running mission
import asyncio

import asyncclick as click

from fly.core.flightSession import FlightSession
from fly.core.resumeManager import ResumeManager
from fly.interfaces.cli.session import (
    remember_setting,
    require_drone,
    require_mission,
    resolve_data_dir_paths,
    resolve_setting,
    state_file_option,
)


async def _resolve_resume(rm: ResumeManager, do_resume: bool | None) -> None:
    # checks state_file for interrupted flight
    if not rm.load():
        return

    print(f"-- Found an interrupted flight: last_waypoint={rm.last_waypoint}")
    if do_resume is None:
        ans = await asyncio.to_thread(input, "Resume from there instead of starting fresh? y/n: ")
        do_resume = ans.strip().lower() == "y"

    if do_resume:
        print(f"-- Will resume at waypoint {rm.resume_index()}.")
    else:
        print("-- Starting fresh; prior resume state will be overwritten as this flight progresses.")
        rm.last_waypoint = -1
        rm.survey_complete = False


async def _build_vision_pipeline(drone, rtsp_url, image_dir, poi_registry, flight_log, model_path, confidence, data_dir):
    # does not start anything: FlightSession's job
    if not rtsp_url:
        return None, None, None, None

    try:
        from fly.vision.rf_detr_nano import NanoDetector  # noqa existence check only
    except ImportError as e:
        print(f"-- Vision dependencies not installed ({e}); flying without live detection.")
        return None, None, None, None

    from fly.comms.streamCapture import StreamCapture
    from fly.logging.flightLog import FlightLog
    from fly.poi.poiManager import POIManager
    from fly.vision.gcsPipeline import GCSPipeline

    capture = StreamCapture(drone, rtsp_url, image_dir)
    if not await capture.open():
        print(f"-- Could not open {rtsp_url}; flying without live detection.")
        return None, None, None, None

    poi_manager = POIManager(registry_path=poi_registry)
    await poi_manager.load()
    flog = FlightLog(flight_log)
    await flog.load_existing()
    pipeline = GCSPipeline(asyncio.Queue(), poi_manager, model_path=model_path, confidence=confidence, flight_log=flog)

    # configuration produced working stream: remembering
    remember_setting("rtsp-url", rtsp_url)
    remember_setting("model-path", model_path)
    remember_setting("confidence-threshold", confidence)
    remember_setting("data-dir", data_dir)

    return capture, pipeline, poi_manager, flog

@click.command(name="run", help="Fly a full mission, optionally with the live vision pipeline, and land.")
@click.option("--file", "file_", required=True, type=click.Path(exists=True), help="Mission JSON file.")
@click.option("--port", help="Connection port. Defauls to the last-used port.")
@click.option("--rtsp-url", default=None, help="Video stream URL, or a local video file for testing. Defaults to the last-used stream; omit entirely and clear the saved one with `fly config wipe` to fly without the vision pipeline.")
@click.option("--data-dir", default=None, help="Base directory for image-dir, poi-registry, flight-log, resume-state defaults. Defaults to last-used directory, or '.' if none is on record.")
@click.option("--image-dir", default=None, help="[default: <data-dir>/captured_images]")
@click.option("--poi-registry", default=None, help="[default: <data-dir>/poi_registry.json]")
@click.option("--flight-log", default=None, help="[default: <data-dir>/flight_log.jsonl]")
@state_file_option
@click.option("--model", "model_path", default=None, help="Custom detector weights (.pth). Defaults to last-used, or COCO weights if none are on record.")
@click.option("--confidence", type=float, default=None, help="Detection confidence threshold. Defaults to the last-used value, or 0.5.")
@click.option("--takeoff-alt", type=float, default=None, help="Arm and take off to this altitude first. Omit if the drone is already airborne or if the autopilot auto-takeoffs on mission start.")
@click.option("--resume/--fresh", "do_resume", default=None, help="Resume a prior interrupted flight if one is on record. Default: ask.")
@click.option("--land-on-finish/--rtl-on-finish", "land_on_finish", default=None, help="Override the mission file's RTL-after-mission flag.")
async def run(file_, port, rtsp_url, data_dir, image_dir, poi_registry, flight_log, state_file, model_path, confidence, takeoff_alt, do_resume, land_on_finish):
    m = require_mission(file_)

    # resolve
    paths = resolve_data_dir_paths(
        data_dir, image_dir=image_dir, poi_registry=poi_registry, flight_log=flight_log, state_file=state_file
    )
    data_dir, image_dir = paths["data_dir"], paths["image_dir"]
    poi_registry, flight_log, state_file = paths["poi_registry"], paths["flight_log"], paths["state_file"]

    rtsp_url = resolve_setting(rtsp_url, "rtsp-url", None)
    model_path = resolve_setting(model_path, "model-path", None, quiet=True)
    confidence = resolve_setting(confidence, "confidence-threshold", 0.5, quiet=True)

    # resume check
    rm = ResumeManager(state_file=state_file)
    await _resolve_resume(rm, do_resume)

    # connect
    drone = await require_drone(port)

    # optional vision pipeline building
    capture, pipeline, poi_manager, flog = await _build_vision_pipeline(
        drone, rtsp_url, image_dir, poi_registry, flight_log, model_path, confidence, data_dir
    )

    # hand everything to FlightSession
    session = FlightSession(
        drone=drone,
        mission=m,
        resume=rm,
        capture=capture,
        pipeline=pipeline,
        poi_manager=poi_manager,
        flight_log=flog,
        takeoff_alt=takeoff_alt,
        land_on_finish=land_on_finish
    )
    finished_cleanly = await session.run()

    status = session.status()
    if pipeline is not None:
        print(f"-- Vision stats: {status['pipeline_stats']}")
    if poi_manager is not None:
        print(f"-- POIs on record: {status['poi_count']}")

    if not finished_cleanly:
        raise SystemExit(1)
