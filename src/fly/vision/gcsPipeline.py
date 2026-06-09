import asyncio
from concurrent.futures import ProcessPoolExecutor

# ...
from fly.comms.protocol import ImagePayload

# module level so ProcessPoolExecutor can pickle
def _detect(image_path: str, model_path: str | None, confidence: float) -> list[dict]:
    # runs NanoDetector in a worker process. returns detection list

class GCSPipeline:
    # reads (ImagePayload, Path) pairs off notify_queue, runs detection in a process pool, and forwards hits to poiManager

    def __init__(self, notify_queue: asyncio.Queue, poi_manager: POIManager,
        model_path: str | None = None,
        confidence: float = .5,
        max_workers: int = 2):
        self.queue = notify_queue
        self.pois = poi_manager
        self.model_path = model_path
        self.confidence = confidence
        self._pool = ProcessPoolExecutor(max_workers = max_workers)
        self._running = False
        self. stats = {"processed": 0, "detections": 0, "errors": 0}

    async def start(self):
        # runs until stop() is called. Launch asyncio.create_task(pipeline.start())

    async def stop(self):
        self._running = False
        self._pool.shutdown(wait=False)
