# orchestrates one full survey flight:
# upload mission, run the vision pipeline (if configured), checkpoint progress,
# and land/RTL when it finishes

import asyncio

from fly.core.mission import Mission
from fly.core.resumeManager import FlightPhase, ResumeManager

DEFAULT_POLL_INTERVAL_S = 2


class FlightSession:
    def __init__(
        self,
        *,
        drone,  # fly.core.drone.Drone is supposed to be already connected
        mission: Mission,  # already supposed to be parsed from a mission file
        resume: ResumeManager,  # already .load()-ed by the caller, so can_resume() is accurate
        capture=None,  # fly.comms.streamCapture.StreamCapture | None, already .open()-ed
        pipeline=None,  # fly.vision.gcsPipeline.GCSPipeline | None
        poi_manager=None,  # fly.poi.poiManager.POIManager | None, already .load()-ed
        flight_log=None,  # fly.logging.flightLog.FlightLog | None, already .load_existing()-ed
        takeoff_alt: float | None = None,
        land_on_finish: bool | None = None,  # None = defer to the mission file's own RTL flag
        poll_interval_s: float = DEFAULT_POLL_INTERVAL_S
    ):
        self.drone = drone
        self.mission = mission
        self.resume = resume
        self.capture = capture
        self.pipeline = pipeline
        self.poi_manager = poi_manager
        self.flight_log = flight_log
        self.takeoff_alt = takeoff_alt
        self.land_on_finish = land_on_finish
        self.poll_interval_s = poll_interval_s

        self._watcher_task: asyncio.Task | None = None
        self._pipeline_task: asyncio.Task | None = None
        self._running = False
        self._interrupted = False
        self._resume_index = 0

# -- lifecycle

    async def run(self) -> bool:
        # returns True on a clean finish, False if interrupted (ctrl+c)
        # resume state is left on disk in False case so a rerun with the same
        # ResumeManager state file picks up where it left off
        self._resume_index = self.resume.resume_index() if self.resume.can_resume() else 0

        if self.takeoff_alt:
            print(f"-- Taking off to {self.takeoff_alt}m...")
            await self.drone.takeoff(self.takeoff_alt)
        elif self._resume_index == 0:
            print(
                "-- No takeoff altitude given; assuming the drone is already"
                "armed/airborne, or that the autopilot auto-takeoffs on mission start."
            )

        print(
            f"-- Uploading mission ({len(self.mission.waypoints)} waypoint(s),"
            f"RTL-after-mission={self.mission.RTL})..."
        )
        await self.mission.upload_mission(self.drone)

        if self._resume_index > 0:
            print(f"-- Seeking to waypoint {self._resume_index}...")
            await self.mission.set_current_mission_target(self.drone, self._resume_index)

        self.resume.transition(FlightPhase.SURVEY)

        self._running = True
        self._interrupted = False
        if self.capture is not None and self.pipeline is not None:
            self._pipeline_task = asyncio.create_task(self.pipeline.start(), name="gcs_pipeline")
            self._watcher_task = asyncio.create_task(
                self.capture.watch_and_capture(self.pipeline.queue), name="capture_watcher"
            )
            print("-- Vision pipeline running alongside the mission.")

        await self.mission.start_mission(self.drone)
        print("-- Mission started.")

        try:
            await self._wait_for_completion()
        except (KeyboardInterrupt, asyncio.CancelledError):
            self._interrupted = True
            print("-- Interrupted. Resume state is saved; rerun with --resume to continue from here.")
        finally:
            await self._stop_background_tasks()

        self._running = False
        if self._interrupted:
            return False

        print("-- Mission complete.")
        self.resume.transition(FlightPhase.RTH)

        should_rtl = self.mission.RTL if self.land_on_finish is None else not self.land_on_finish
        if should_rtl:
            print("-- Returning to launch.")
            await self.drone.return_to_home()
        else:
            print("-- Landing at current position.")
            await self.drone.land()

        self.resume.path.unlink(missing_ok=True)
        print("-- Done. Resume state cleared.")
        return True

    async def stop(self):
        # request an early, graceful stop
        self._interrupted = True
        await self._stop_background_tasks()

# -- internals

    async def _wait_for_completion(self):
        last_seen = self._resume_index - 1
        while True:
            if await self.mission.is_mission_finished(self.drone):
                return
            progress = await self.mission.get_mission_progress(self.drone)
            if progress:
                current, total = progress
                if current != last_seen:
                    last_seen = current
                    self.resume.waypoint_done(current)
                    print(f"-- waypoint {current}/{total}")
                await asyncio.sleep(self.poll_interval_s)

    async def _stop_background_tasks(self):
        for task in (self._watcher_task, self._pipeline_task):
            if task is not None and not task.done():
                task.cancel()
        pending = [t for t in (self._watcher_task, self._pipeline_task) if t is not None]
        if pending:
            await asyncio.gather(*pending, return_exceptions=True)
        if self.pipeline is not None:
            await self.pipeline.stop()
        if self.capture is not None:
            await self.capture.close()

# -- introspection
    def status(self) -> dict:
        # a single consolidated snaption of the session
        # safe to call at any point, including mid-flight from elsewhere in
        # the same process (e.g. a GUI polling this for a live progress display),
        # since it only reads already-in-memory state and never touches drone or disk
        info = {
            "running": self._running,
            "phase": self.resume.phase.value,
            "last_waypoint": self.resume.last_waypoint,
            "total_waypoints": len(self.mission.waypoints),
            "resume_index": self._resume_index,
            "vision_enabled": self.pipeline is not None
        }
        if self.flight_log is not None:
            info["log_entries"] = len(self.flight_log.all_entries())
        if self.poi_manager is not None:
            info["poi_count"] = len(self.poi_manager.get_all())
        if self.pipeline is not None:
            info["pipeline_stats"] = dict(self.pipeline.stats)
        return info
