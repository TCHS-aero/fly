from mavsdk import System 
import asyncio
import sys


drone = System()
task_list = []
async def main(): #create drone variable so we can refer to the system's functions


    await drone.connect('udpin://0.0.0.0:14540') #connect to the drone

    async for checks in drone.core.connection_state(): #uses async for loop to continuosly check if drone is successfully connected
        if checks:
            print("successful connection")
            break #if true break out of loop and move on
        else:
            print("unsuccessful connection")
            continue #if false retry connection

    async for checks in drone.telemetry.health(): #async for loop that grabs the drone's health, specifically global position and home position
        if checks.is_global_position_ok and checks.is_home_position_ok: #if the two are true that means the drone has passed the mandatory health check
            print('successful health checkup')
            break #break out of loop to continue
        else:
            print('unsuccessful health checkup')
            continue #stay in loop to recheck

    await drone.action.arm() #arms the drone for takeoff
    await drone.action.takeoff() #takeoff

    async for checks in drone.telemetry.in_air(): #this async for loop starts a 30 second countdown the second the drone lifts up from the ground by checking if the drone is in the air
        if checks:
            print("drone is in air, starting countdown", checks)
            batteryd = asyncio.create_task(battery()) #when the drone is in the air, the code starts 2 tasks, one for collecting battery telem. and one for coordinate telem.
            coordinatesd = asyncio.create_task(coordinates())
            task_list = [batteryd, coordinatesd]#create a list to contain these tasks for later cancellation

            await asyncio.sleep(30)
            break #breaks out of loop to continue
        else:
            print("drone is not in air, can not start countdown", checks)
            continue #continue checking if the drone is on the ground

    await drone.action.land() #after checking land the drone

    async for air in drone.telemetry.in_air():
        if air:
            continue #if air is true that means the drone has not landed yet, can not cancell tasks yet
        else:
            break #if it's on the ground break out of loop to continue

    close_tasks(task_list) 

def close_tasks(task_list): #pass the task list and use a for loop to close tasks
    for i in task_list:
        i.cancel()
    print('debug1')


async def coordinates():
    oldlon = None #set old coordinates for first time
    oldlat = None
    oldalt = None
    async for coords in drone.telemetry.position(): #have a constant stream of information in a async for loop
        if coords.latitude_deg and coords.longitude_deg and coords.absolute_altitude_m: #grabs the wanted information
            newlat = coords.latitude_deg #sets new latitude, longitude, and altitude
            newlon = coords.longitude_deg 
            newalt = round(coords.absolute_altitude_m, 1)
            if oldlon != newlon and oldlat != newlat and oldalt != newalt: #compares the new and old data to see if the data is a duplicate of the old information
                print(f'longitude: {newlon}, latitude: {newlat}, altitude relative to sea level(m): {newalt}')
                oldlon, oldlat, oldalt = newlon, newlat, newalt #set new info to old info to repeat the proccess
                await asyncio.sleep(1) #wait 1 second to prevent information overload
            else:
                continue #if the old and new info are the same it loops back and continues until the data has changed


async def battery():
    oldtime = None #same logic can be applied here
    oldpercent = None
    async for battery in drone.telemetry.battery():
        if battery.time_remaining_s and battery.remaining_percent:
            newtime = round(battery.time_remaining_s,1)
            newpercent = round(battery.remaining_percent,1)
            if oldtime != newtime and oldpercent != newpercent:
                print(f"remaining percentage: {newpercent}, time remaining(s): {newtime}")
                oldtime, oldpercent = newtime, newpercent
                await asyncio.sleep(1)
            else:
                continue

            


if __name__ == "__main__":
    asyncio.run(main()) #run the script
    sys.exit("goodbye fellers") 