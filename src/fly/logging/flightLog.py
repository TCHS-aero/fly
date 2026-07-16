import asyncio
import aiofiles
import json
from pathlib import Path
from fly.comms.protocol import ImagePayload

class FlightLog:
    # append-only jsonl file linking image filenames to GPS coordinates;
    # in-memory dictionary for O(1) filename lookup

    def __init__(self, log_path:str):
        self.path = Path(log_path)
        self._lock = asyncio.Lock()
        self._index: dict[str, dict] = {} # filename -> entry
        self._seq = 0

    async def load_existing(self):
        # reads existing log, rebuilds index, sets seq counter. Skips bad lines
        if not self.path.exists():
            return
        async with aiofiles.open(self.path, "r") as f:
            async for raw_line in f:
                line = raw_line.strip()
                if not line:
                    continue
                try:
                    entry = json.loads(line)
                    self._index[entry["filename"]] = entry
                    seq = entry.get("seq", -1)
                    if seq >= self._seq:
                        self._seq = seq + 1
                except (json.JSONDecodeError, KeyError):
                    continue # skips corrupt lines silently


    async def append(self, payload: ImagePayload) -> dict:
        # appends one entry and updates the index
        # thread-safe via asyncio.Lock
        async with self._lock:
            entry = {
                "seq": self._seq,
                "ts": payload.ts,
                "lat": payload.lat,
                "lon": payload.lon,
                "alt_rel": payload.alt_rel,
                "wp_index": payload.wp_index,
                "phase": payload.phase,
                "filename": payload.filename,
            }
            self._seq += 1
            self._index[payload.filename] = entry
            async with aiofiles.open(self.path, "a") as f: # "a": append mode
                await f.write(json.dumps(entry) + "\n")
        return entry

    def lookup(self, filename:str) -> dict | None:
        # returns the entry or None
        return self._index.get(filename)

    def all_entries(self) -> list[dict]:
        # returns all entries in seq order
        return sorted(self._index.values(), key = lambda e: e["seq"])
