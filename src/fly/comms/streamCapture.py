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
        self._cap = await asyncio.to_thread(cv2.VideoCapture(self.rtsp_url))

        # Creates image_dir if it doesn't exist.
        self.image_dir.mkdir(parents = True, exist_ok = True):

        # Checks if image_dir exists and returns True, otherwise returns False
        if self.image_dir.exists():
            return True
        return False

    async def capture_frame(
        self, wp_index: int, phase: str = "survey"
    ) -> tuple[ImagePayload, Path] | None:
        # Note: must wrap OpenCV logic in a thread pool `await asyncio.to_thread(...)`, just like what is in open()

        # 2. cap.read() - live frame from HM30
        ret, frame = await asyncio.to_thread(self._cap.read())

        # Returns None if the stream is unavaliable (caller can retry next tick)
        if not self._cap.isOpened():
            print("-- Video stream is not opened.")
            return None

        if not ret:
            print("-- Failed to capture frame.")
            return None

        # grabs current frame and pairs it with live telemetry
        ts = ImagePayload.now_ts()

        # creates image path under self.image_dir
        image_path = self.image_dir / f"frame_{ts}.png"

        # 3. drone.current_position() - lat, lon, alt_rel from MAVSDK
        lat, lon, alt_rel = self.drone.current_position()

        # 4. cv2.imwrite(path, frame) - save as JPEG; filename derived from ts
        await asyncio.to_thread(cv2.imwrite(image_path, frame))

        # 5. Return (ImagePayload and Path)
        payload = ImagePayload(
            ts = ts,
            lat = lat,
            lon = lon,
            alt_rel = alt_rel,
            wp_index = wp_index,
            phase = phase,
            image_path = image_path
        )

        return (payload, image_path)


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
