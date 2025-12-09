import asyncio


class telemetry:
    def __init__(self, drone):
        self.drone = drone
        self.tasks = []

    def _subscribe(self, gen_fn, name, callback):
        """
        Generic wrapper for telemetry async generators.
        Safely handles cancellation and generator closing.
        """

        async def runner():
            agen = gen_fn()
            try:
                async for data in agen:
                    callback(data)
            except asyncio.CancelledError:
                try:
                    await agen.aclose()
                except Exception:
                    pass
                raise
            except Exception as e:
                print(f"-- {name} error:", e)
                try:
                    await agen.aclose()
                except Exception:
                    pass

        task = asyncio.create_task(runner(), name=name)
        self.tasks.append(task)

    def log_battery(self):
        old_data = None

        def callback(log):
            nonlocal old_data
            new_data = round(log.remaining_percent, 1)
            if old_data != new_data:
                old_data = new_data
                print(f"-- Remaining battery: {new_data}%")
                print(f"-- Estimated time until 0%: {log.time_remaining_s}(s)")

        self._subscribe(
            self.drone.telemetry.battery,
            "log_battery",
            callback,
        )

    def log_altitude(self):
        old_data = None

        def callback(log):
            nonlocal old_data
            new_data = round(log.altitude_relative_m, 1)
            if old_data != new_data:
                old_data = new_data
                print(f"-- Altitude in meters from home position: {new_data}(m)")

        self._subscribe(
            self.drone.telemetry.altitude,
            "log_altitude",
            callback,
        )

    async def cancel_running_tasks(self):
        for task in self.tasks:
            task.cancel()
            print(f'-- Closed coroutine: "{task.get_name()}"')
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.tasks.clear()


class health_check:
    def __init__(self, drone):
        self.drone = drone

    async def check_stable_connection(self):
        """
        Halts script execution until a stable connection to the drone is found.
        """

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Found stable connection")
                break
            print("-- Couldn't find a stable connection, retrying...")

    async def position_checks(self):
        """
        Halts script execution until all positional health checks are satisfied.
        This is necessary in order to arm the drone.
        """

        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Health checks satisfied")
                break
