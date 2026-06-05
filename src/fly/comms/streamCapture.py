import asyncio
from pathlib import Path

import cv2

from fly.comms.protocol import ImagePayload


class StreamCapture:
    """
    Opens the HM30 RTSP stream and grabs frames on demand
    Each grab is paired with a live MAVSDK telemetry read so the image
    and its GPS coordinates are matched as closely as possible
    """

    def __init__(self, drone, rtsp_url: str, image_dir: str):
        self.drone = drone
        self.rtsp_url = rtsp_url
        self.image_dir = Path(image_dir)
        self._cap: cv2.VideoCapture | None = None

    async def open(self) -> bool:
        # opens the rtsp stream via cv2.VideoCapture (runs in executor to avoid
        # blocking the event loop during the 1s RTSP handshake)
        # Creates image_dir if it doesn't exist. Returns True on success.
        pass

    async def capture_frame(
        self, wp_index: int, phase: str = "survey"
    ) -> tuple[ImagePayload, Path] | None:
        # grabs current frame and pairs it with live telemetry
        ts = ImagePayload.now_ts()
        # 2. cap.read() - live frame from HM30
        # 3. drone.current_position() - lat, lon, alt_rel from MAVSDK
        # 4. cv2.imwrite(path, frame) - save as JPEG; filename derived from ts
        # 5. Return (ImagePayload.path)
        # Returns None if the stream is unavaliable (caller can retry next tick)
        # Note: must wrap OpenCV logic in a thread pool `await asyncio.to_thread(...)`, just like what is in open()
        pass

    async def watch_and_capture(self, notify_queue: asyncio.Queue):
        # Background task: subscribes to mission_progress() and calls
        # capture_frame() on each new waypoint, putting results on notify_queue
        # run as: asyncio.create_task(capture.watch_and_capture(queue))
        print("-- Starting waypoint capture watcher.")
        last_seen_waypoint = -1

        # loop runs whenever drone sends progress update
        async for progress in self.drone.drone.mission.mission_progress():
            if progress.current > last_seen_waypoint:
                print(
                    f"-- Reached new waypoint #{progress.current}. Capturing frame..."
                )

                last_seen_waypoint = progress.current

                capture_result = await self.capture_frame(
                    wp_index=progress.current, phase="survey"
                )
                if capture_result is not None:
                    # put result onto queue; another part of program will listen
                    payload, img_path = capture_result
                    try:
                        notify_queue.put_nowait(payload)
                    except asyncio.QueueFull:
                        print(
                            f"-- Queue full, dropping frame. Dropping image payload for waypoint {progress.current}"
                        )

        print("Watcher stopped.")

    async def close(self):
        if self._cap:
            self._cap.release()
