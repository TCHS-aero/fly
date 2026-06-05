import asyncio, aiofiles, json
from pathlib import Path
from datetime import datetime, timezone

class FlightLog:
    # append-only jsonl file linking image filenames to GPS coordinates;
    # in-memory dictionary for O(1) filename lookup

    def __init__(self, log_path:str):
        self.path = Path(log_path)
        self._lock = asyncio.Lock()
        self._index: dict[str, dict] = {} # filename, entry
        self._seq = 0

    async def load_existing(self):
        # reads existing log, rebuilds index, sets seq counter. Skips bad lines

    async def append(self, payload: "ImagePayload") -> dict:
        # appends one entry and updates the index
        # thread-safe via asyncio.Lock

    def lookup_by_image(self, filename:str) -> dict | None:
        # returns the entry or None

    def get_all_entries(self) -> list[dict]:
        # returns all entries in seq order
