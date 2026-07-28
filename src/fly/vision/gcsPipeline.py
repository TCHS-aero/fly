import asyncio
import math
from concurrent.futures import ProcessPoolExecutor

from fly.comms.protocol import ImagePayload
from fly.logging.flightLog import FlightLog
from fly.poi.poiManager import POIManager
from fly.utils.geo import Point, pixel_to_ground

_worker_detector = None
_worker_model_path: str | None = None

# module level so ProcessPoolExecutor can pickle
def _detect(image_path: str, model_path: str | None, confidence: float) -> tuple[list[dict], tuple[int, int]]:
    # runs NanoDetector in a worker process. returns detection list
    global _worker_detector, _worker_model_path

    if _worker_detector is None or _worker_model_path != model_path:
        from fly.vision.rf_detr_nano import NanoDetector
        _worker_detector = NanoDetector(confidence_threshold=confidence, model_path=model_path)
        _worker_detector.load_model()
        _worker_model_path = model_path

    image = _worker_detector.load_image(image_path)
    detections, _, _ = _worker_detector.detect(image) # _, _ is raw detection data and duration; unnecessary
    return detections, image.size # list[dict] is already JSON-serialisable
# sentinel put on the queue by stop() to unblock a queue.get()
# without _STOP, the queue would not have a chance to detect that _running is False, since it's waiting for the next item
_STOP = object()

class GCSPipeline:
    # reads (ImagePayload, Path) pairs off notify_queue, runs detection in a process pool, and forwards hits to poiManager

    def __init__(
        self,
        notify_queue: asyncio.Queue[tuple[ImagePayload, str] | object], # object() is for _STOP
        poi_manager: POIManager,
        model_path: str | None = None,
        confidence: float = .5,
        max_workers: int = 2,
        flight_log: FlightLog | None = None
    ):
        self.queue = notify_queue
        self.pois = poi_manager
        self.model_path = model_path
        self.confidence = confidence
        self._pool = ProcessPoolExecutor(max_workers = max_workers)
        self._running = False
        self.stats = {"processed": 0, "detections": 0, "errors": 0}
        # optional: every frame that passes through the pipeline gets appended here,
        # regardless of whether a detection was found, so logQuery can later map any filename
        # back to where/when it was taken
        self.flight_log = flight_log

    async def start(self):
        # runs until stop() is called.
        self._running = True
        loop = asyncio.get_running_loop()
        while self._running:
            item = await self.queue.get() # unpacks input from streamCapture.watch_and_capture()
            if (
                item is _STOP
                or not isinstance(item, tuple) # prevents type checking error while unpacking item later
            ):
                self.queue.task_done()
                break
            payload, image_path = item
            try:
                if self.flight_log is not None:
                    await self.flight_log.append(payload)

                detections, image_wh = await loop.run_in_executor(
                    self._pool, _detect, str(image_path), self.model_path, self.confidence
                )
                self.stats["processed"] += 1
                for det in detections:
                    ground_pos = self._project_to_ground(det, image_wh, payload)
                    await self.pois.add_detection(
                        ground_pos,
                        det["confidence"],
                        payload.filename,
                    )
                    self.stats["detections"] += 1
            except Exception as e:  # noqa , error is outputted anyway
                self.stats["errors"] += 1
                print(f"Pipeline error on {payload.filename}: {e}")
            finally:
                self.queue.task_done()

    def _project_to_ground(self, det: dict, image_wh: tuple[int, int], payload: ImagePayload) -> Point:
        if math.isnan(payload.heading_deg) or payload.alt_rel <= 0:
            print(
                f"-- {payload.filename}: heading/altitude unusable "
                f"(heading={payload.heading_deg}, alt={payload.alt_rel}); "
                "falling back to raw drone GPS position for this detection."
            )
            return payload.pos

        return pixel_to_ground(
            detected_xy=(det["center"]["x"], det["center"]["y"]),
            image_wh=image_wh,
            drone_pos=payload.pos,
            alt_m=payload.alt_rel,
            heading_deg=payload.heading_deg,
        )

    async def stop(self):
        self._running = False
        await self.queue.put(_STOP)
        self._pool.shutdown(wait=False)
