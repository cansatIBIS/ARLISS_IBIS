import asyncio
from Library.pixhawk import Pixhawk
from Library.logger_lib import logger_info, logger_debug
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

# parameters---------------------
waypoint = [40.19373, 140.05923]
altitude = 5
speed = 5
#--------------------------------

async def run():

    main_coroutines = [
        pixhawk.cycle_flight_mode(),
        pixhawk.cycle_position_lat_lng(), 
        pixhawk.cycle_lidar(),
        pixhawk.cycle_show()
        ]
    
    await pixhawk.connect()
    await drone.mission.clear_mission()
    logger_info.info("Mission cleared")
    await pixhawk.upload_mission(waypoint, altitude, speed)

    # await pixhawk.health_check()

    await pixhawk.arm()

    

    await pixhawk.start_mission()

    await asyncio.gather(*main_coroutines)

    while True:
        await asyncio.sleep(1)
        mission_finished = await drone.mission.is_mission_finished()
        if mission_finished:
            logger_info.info("mission_finished")
            break
    
    await drone.mission.clear_mission()

    await pixhawk.upload_mission(waypoint, altitude, speed)

    await pixhawk.arm()

    await pixhawk.start_mission()

    while True:
        await asyncio.sleep(1)
        mission_finished = await drone.mission.is_mission_finished()
        if mission_finished:
            logger_info.info("mission_finished")
            break

    logger_info.info("Landing")
    await drone.action.land()
    await asyncio.sleep(10)
    logger_info.info("Landed")

    


if __name__ == "__main__":

    pixhawk = Pixhawk()
    drone = System()
    asyncio.run(run())