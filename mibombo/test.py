from mavsdk import System
from mavsdk.offboard import PositionNedYaw
from fly.core.mission import Mission
import asyncio

drone = System()
async def main(): 
    missionTEST = Mission('mission_files/mission_waypoints.json')

    await drone.connect('udpin://0.0.0.0:14540')

    async for checks in drone.core.connection_state(): 
        if checks:
            print("successful connection")
            break 
        else:
            print("unsuccessful connection")
            continue 

    async for checks in drone.telemetry.health():
        if checks.is_global_position_ok and checks.is_home_position_ok: 
            print('successful health checkup')
            break 
        else:
            print('unsuccessful health checkup')
            continue 
    
    await drone.action.arm()
    print("armed")

    try:
        await missionTEST.clear_mission(drone)
        print('clearing old mission if there was an old mission')

        await missionTEST.return_to_launch_after_mission_completion(drone, True)
        print('drone is set to return after mission completion.')

        missionTEST.convert_mission_items_to_plan()
        print('converting json file\'s mission items to mission_plan')

        await missionTEST.upload_the_mission(drone)
        print('uploaded mission!') 

        await missionTEST.start_mission_plan(drone)
        print('start mission')

        mission_progress = asyncio.create_task(missionTEST.check_mission_progress(drone))
        await mission_progress

        
    except Exception as e:
        print(e)


    '''await drone.action.land() 
    print('land')

    async for air in drone.telemetry.in_air():
        if air:
            continue
        else:
            break 

    print('disarm')'''

if __name__ == "__main__":
    asyncio.run(main()) 