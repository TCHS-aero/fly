import json
from enum import Enum
from pathlib import Path
from datetime import datetime, timezone

from mavsdk.calibration import Calibration

class FlightPhase(Enum):
    IDLE = "idle"
    SURVEY = "survey"
    CALIBRATION = "calibration"
    RTH = "rth"

class ResumeManager:
    # write state to resume_state.json on every transition

    def __init__(self, state_file:str = "resume_state.json"):
        self.path = Path(state_file)
        self.phase = FlightPhase.IDLE
        self.last_waypoint = -1
        self.survey_complete = False

    def load(self) -> bool:
        # loads previous state, returns true if interrupted mission found

    def save(self):
        # write then rename

    def transition(self, phase: FlightPhase):
        self.phase = phase
        self.save()

    def waypoint_done(self, index:int):
        # called on each MAVSDK mission_progress tick
        self.last_waypoint = index
        self.save()

    def survey_done(self):
        self.survey_complete = True
        self.transition(FlightPhase.CALIBRATION)

    def can_resume(self) -> bool:
        return self.last_waypoint >= 0 and not self.survey_complete

    def resume_index(self) -> int:
        return max(0, self.last_waypoint)
