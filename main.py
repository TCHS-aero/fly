import asyncio

from mavsdk import System

from modules import health_check, telemetry


async def wait_until_landed(drone):
    """
    Halts script execution until the drone is completely stationary on the ground.
    """

    async for status in drone.telemetry.in_air():
        if not status:
            print("-- Landed")
            break


async def main():
    """
    The main code block. Returns recommended telemetry data and takes off for 15 seconds before landing.
    """

    takeoff_altitude = 5

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    telemetry_func = telemetry(drone)
    health_checks = health_check(drone)

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

    await asyncio.sleep(15)

    await drone.action.land()
    print("-- Landing")

    await wait_until_landed(drone)

    await drone.action.disarm()
    print("-- Disarmed")

    print("-- Closing all running tasks...")
    await telemetry_func.cancel_running_tasks()

    print("Type exit to quit the program.")
    while input("Input: ") != "exit":
        pass


if __name__ == "__main__":
    asyncio.run(main())
