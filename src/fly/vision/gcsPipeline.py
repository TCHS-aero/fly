import asyncio
from concurrent.futures import ProcessPoolExecutor
from fly.poi.poiManager import POIManager
from fly.comms.protocol import ImagePayload

_worker_detector = None
_worker_model_path: str | None = None

# module level so ProcessPoolExecutor can pickle
def _detect(image_path: str, model_path: str | None, confidence: float) -> list[dict]:
    # runs NanoDetector in a worker process. returns detection list
    global _worker_detector, _worker_model_path

    if _worker_detector is None or _worker_model_path != model_path:
        from fly.vision.rf_detr_nano import NanoDetector
        _worker_detector = NanoDetector(confidence_threshold=confidence, model_path=model_path)
        _worker_detector.load_model()
        _worker_model_path = model_path

    image = _worker_detector.load_image(image_path)
    detections, _, _ = _worker_detector.detect(image) # _, _ is raw detection data and duration; unnecessary
    return detections # list[dict] is already JSON-serialisable

class GCSPipeline:
    # reads (ImagePayload, Path) pairs off notify_queue, runs detection in a process pool, and forwards hits to poiManager

    def __init__(
        self,
        notify_queue: asyncio.Queue[tuple[ImagePayload, str]],
        poi_manager: POIManager,
        model_path: str | None = None,
        confidence: float = .5,
        max_workers: int = 2
    ):
        self.queue = notify_queue
        self.pois = poi_manager
        self.model_path = model_path
        self.confidence = confidence
        self._pool = ProcessPoolExecutor(max_workers = max_workers)
        self._running = False
        self.stats = {"processed": 0, "detections": 0, "errors": 0}

    async def start(self):
        # runs until stop() is called.
        self._running = True
        loop = asyncio.get_running_loop()
        while self._running:
            payload, image_path = await self.queue.get()
            try:
                detections = await loop.run_in_executor(
                    self._pool, _detect, str(image_path), self.model_path, self.confidence
                )
                self.stats["processed"] += 1
                for det in detections:
                    await self.pois.add_detection(
                        payload.pos,
                        det["confidence"],
                        payload.filename,
                    )
                    self.stats["detections"] += 1
            except Exception as e:
                self.stats["errors"] += 1
                print(f"Pipeline error on {payload.filename}: {e}")
            finally:
                self.queue.task_done()

    async def stop(self):
        self._running = False
        self._pool.shutdown(wait=False)
