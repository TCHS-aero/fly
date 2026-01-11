import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, 0, 0))
    await drone.offboard.start()
    while ans := input("Input Direction to move in meters. (Direction North, Direction East, Direction Down, Yaw in Degrees. Use negative values for inverse directions.)\nInput:"):
        values = ans.split()
        values = [float(i) for i in values]
        try:
            await drone.offboard.set_position_ned(PositionNedYaw(*ans))
        except Exception as e:
            print(e)

asyncio.run(main())
