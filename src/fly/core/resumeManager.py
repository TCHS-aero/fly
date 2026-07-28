import contextlib
import json
import os
import tempfile
from datetime import UTC, datetime
from pathlib import Path


class ResumeManager:
    # write state to resume_state.json on every transition

    def __init__(self, state_file: str = "resume_state.json"):
        self.path = Path(state_file)
        self.last_waypoint = -1
        self.survey_complete = False

    def load(self) -> bool:
        # loads previous state, returns true if interrupted mission found
        try:
            with open(self.path) as f:
                data = json.load(f)
            self.last_waypoint = int(data.get("last_waypoint", -1))
            self.survey_complete = bool(data.get("survey_complete", False))
            return self.last_waypoint >= 0 and not self.survey_complete
        except (FileNotFoundError, json.JSONDecodeError, KeyError, ValueError):
            return False

    def save(self):
        # write then rename
        data = {
            "last_waypoint": self.last_waypoint,
            "survey_complete": self.survey_complete,
            "saved_at": datetime.now(UTC).isoformat(),
        }
        # mkstemp in the same directory guarantees os.replace is on one filesystem (/tmp uses RAM)
        self.path.parent.mkdir(parents = True, exist_ok = True) # creates the directory if it doesn't exist (on first time save)
        fd, tmp = tempfile.mkstemp(dir=self.path.parent, prefix=".resume_tmp_")
        try:
            with os.fdopen(fd, "w") as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, self.path)  # no risk of half-written file since self.path will point to data in tmp; tmp is deleted
        except Exception:
            with contextlib.suppress(OSError):
                os.unlink(tmp)
            raise

    def waypoint_done(self, index: int):
        # called on each MAVSDK mission_progress tick
        self.last_waypoint = index
        self.save()

    def can_resume(self) -> bool:
        return self.last_waypoint >= 0 and not self.survey_complete

    def resume_index(self) -> int:
        return max(0, self.last_waypoint)
