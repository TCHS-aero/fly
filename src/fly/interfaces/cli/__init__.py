"""
Command groups:

    aero flight ...       connect / status / takeoff / land / return
    aero move ...         manual NED nudges
    aero mission ...      load / upload / start / pause / status / download
    aero mission edit ... mid-flight mission append / insert / remove (MissionEditor)
    aero resume ...       inspect/clear saved crash-recovery state
    aero log ...          query the flight log
    aero poi ...          POI registry, approach-waypoint generation
    aero config ...       show/wipe saved connections settings
    aero run              flagship end-to-end flight
    aero detect-image     run the detector on a single local image (no drone)
    aero capture-test     smoke-test StreamCapture (single frame, needs drone)

Registered under the `aero` console script (see pyproject.toml).
"""
import asyncio
import json

import asyncclick as click

from fly.core.data_manager import pull_data
from fly.interfaces.cli.flight import flight, move
from fly.interfaces.cli.log import log
from fly.interfaces.cli.mission import mission
from fly.interfaces.cli.poi import poi
from fly.interfaces.cli.resume import resume
from fly.interfaces.cli.run import run
from fly.interfaces.cli.session import require_drone, require_nano_detector


@click.group(help="Fly: drone mission CLI (connect, fly, edit missions live, run the vision pipeline).")
def cli():
    pass

@click.group(help="Show or wipe locally-saved connection settings.")
def config():
    pass


@config.command(name="show", help="Show the saved port/history/progress settings.")
def config_show():
    data = pull_data()
    if not data:
        print("-- No settings on record.")
        return
    for k, v in data.items():
        print(f"   {k}: {v}")

# vision diagnostics ------
@click.command(name="detect-image", help="Run the detector on a single local image file (no drone needed).")
@click.option("--image", "image_path", required=True, type=click.Path(exists=True))
@click.option("--model", "model_path", default=None, help="Custom detector weights (.pth). Defaults to last-used, or COCO weights if none are on record.")
@click.option("--confidence", type=float, default=0.5, show_default=True)
@click.option("--out", "out_path", default=None, help="Save an annotated copy of the image here.")
async def detect_image(image_path, model_path, confidence, out_path):
    NanoDetector = require_nano_detector()
    detector = NanoDetector(confidence_threshold=confidence, model_path=model_path)

    def _run():
        detector.load_model()
        image = detector.load_image(image_path)
        detections, raw, inference_time = detector.detect(image)
        if out_path:
            detector.visualize(image, raw, output_path=out_path)
        return detections, inference_time

    detections, inference_time = await asyncio.to_thread(_run)
    print(f"-- {len(detections)} detection(s) in {inference_time * 1000:.1f}ms")
    print(json.dumps(detections, indent=2))


@click.command(name="capture-test", help="Open a stream and grab one frame, paired with drone telemetry.")
@click.option("--port", help="Connection port. Defaults to last-used port.")
@click.option("--rtsp-url", required=True, help="Video stream URL, or a local video file path for bench testing.")
@click.option("--image-dir", default="captured_images", show_default=True)
async def capture_test(port, rtsp_url, image_dir):
    from fly.comms.stream_capture import StreamCapture

    drone = await require_drone(port)

    capture = StreamCapture(drone, rtsp_url, image_dir)
    if not await capture.open():
        print(f"-- Could not open video stream: {rtsp_url}")
        raise SystemExit(1)
    print(f"-- Stream opened: {rtsp_url}")

    try:
        result = await capture.capture_frame(wp_index=0)
        if result is None:
            print("-- Failed to capture a frame.")
            raise SystemExit(1)
        payload, path = result
        print(f"-- Captured {path}")
        print(f"   lat={payload.lat:.6f} lon={payload.lon:.6f} alt={payload.alt_rel}m  heading={payload.heading_deg}")
    finally:
        await capture.close()

# assemble ------------

for command in (flight, move, mission, resume, log, poi, config, run, detect_image, capture_test):
    cli.add_command(command)


def main():
    try:
        cli(_anyio_backend="asyncio")
    except KeyboardInterrupt:
        print("\n-- Interrupted.")
        raise SystemExit(130) from None


if __name__ == "__main__":
    main()
