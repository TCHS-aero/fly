import asyncio

from mavsdk import System

from modules import Checks, TelemetryLogging


async def main():
    """
    The main code block. Returns recommended telemetry data and takes off for 15 seconds before landing.
    """

    # Set default altitude to 5 meters so that when we take off, it takes off only 5 meters above ground
    takeoff_altitude = 5

    # Create class instance of System
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    # Import custom modules
    telemetry_func = TelemetryLogging(drone)
    checks = Checks(drone)

    # Health checks that are required to fly and land drone.
    await checks.check_stable_connection()
    await checks.position_checks()

    # Turning on drone via "arming"
    await drone.action.arm()
    print("-- Armed")

    # Setting default altitude
    print(f"-- Setting takeoff altitude to {takeoff_altitude}")
    await drone.action.set_takeoff_altitude(takeoff_altitude)

    # Taking off
    await drone.action.takeoff()
    print("-- Taking off...")

    # Printing/logging current altitude and battery percentage.
    telemetry_func.log_altitude()
    telemetry_func.log_battery()

    await asyncio.sleep(15)

    # Landing drone safely
    await drone.action.land()
    print("-- Landing")

    # Waiting to land properly before disarming or turning off drone
    await checks.wait_until_landed()

    # disarming
    await drone.action.disarm()
    print("-- Disarmed")

    # Closing all logging coroutines
    print("-- Closing all running tasks...")
    await telemetry_func.cancel_running_tasks()

    # Waiting for user input to close the program
    print("Type exit to quit the program.")
    while input("Input: ") != "exit":
        pass


if __name__ == "__main__":
    asyncio.run(main())
