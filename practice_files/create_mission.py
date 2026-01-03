import asyncio

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

from modules import Checks, TelemetryLogging


async def mission(drone):
    await drone.mission.clear_mission()
    mission_waypoints = []
    mission_waypoints.append(
        MissionItem(
            47.397,
            8.54,
            25,
            10,
            True,
            float("nan"),
            float("nan"),
            MissionItem.CameraAction.NONE,
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            MissionItem.VehicleAction.NONE,
        )
    )
    mission_waypoints.append(
        MissionItem(
            47.400,
            8.6,
            25,
            10,
            True,
            float("nan"),
            float("nan"),
            MissionItem.CameraAction.NONE,
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            MissionItem.VehicleAction.NONE,
        )
    )
    mission = MissionPlan(mission_waypoints)
    print(mission)

    await drone.mission.set_return_to_launch_after_mission(True)

    await drone.mission.upload_mission(mission)

    await drone.mission.start_mission()

    await asyncio.sleep(10)


def close_tasks(tasks):
    for task in tasks:
        task.cancel()


async def main():
    """
    The main code block. Returns recommended telemetry data and takes off for 15 seconds before landing.
    """

    takeoff_altitude = 5

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    telemetry_func = TelemetryLogging(drone)
    health_checks = Checks(drone)

    # Health checks to guarentee proper drone connections.
    await health_checks.check_stable_connection()
    await health_checks.position_checks()

    await drone.action.arm()
    print("-- Armed")

    print(f"-- Setting takeoff altitude to {takeoff_altitude}")
    await drone.action.set_takeoff_altitude(takeoff_altitude)

    await drone.action.takeoff()
    print("-- Taking off...")

    telemetry_func.log_altitude()
    telemetry_func.log_battery()

    Mission = asyncio.create_task(mission(drone))
    tasks = [Mission]

    await asyncio.sleep(12)

    await drone.action.land()
    print("-- Landing")

    await health_checks.wait_until_landed(drone)

    await drone.action.disarm()
    print("-- Disarmed")

    print("-- Closing all running tasks...")
    await telemetry_func.cancel_running_tasks()
    close_tasks(tasks)


if __name__ == "__main__":
    asyncio.run(main())
