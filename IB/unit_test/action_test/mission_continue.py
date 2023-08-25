import sys

ibis_directory = "/home/pi/ARLISS_IBIS/IB/Library"
sys.path.append(ibis_directory)

import asyncio
from pixhawk import Pixhawk
from logger_lib import logger_info
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

# parameters---------------------
fuse_PIN = 0
wait_time = 0
lora_sleep_time = 0
fuse_time = 0
land_timelimit = 0
health_continuous_count = 10
waypoint_lat = 40.19373
waypoint_lng = 140.05923
waypoint_alt = 5
mission_speed = 5
lora_power_Pin = 0
#--------------------------------

async def run():

    drone = System()
    pixhawk = Pixhawk(
                 fuse_PIN,
                 wait_time,
                 lora_sleep_time, 
                 fuse_time,
                 land_timelimit,
                 health_continuous_count,
                 waypoint_lat,
                 waypoint_lng,
                 waypoint_alt,
                 mission_speed,
                 lora_power_Pin
                 )

    main_coroutines = [
        pixhawk.cycle_flight_mode(),
        pixhawk.cycle_position_lat_lng(), 
        pixhawk.cycle_lidar(),
        pixhawk.cycle_show(),
        pixhawk.mission_land()
        ]
    
    await pixhawk.connect()

    await pixhawk.upload_mission()

    # await pixhawk.health_check()

    await pixhawk.arm()

    await pixhawk.clear_mission()

    await pixhawk.start_mission()

    await asyncio.gather(*main_coroutines)

    while True:
        await asyncio.sleep(1)
        mission_finished = await drone.mission.is_mission_finished()
        if mission_finished:
            logger_info.info("mission_finished")
            break
    
    await drone.mission.clear_mission()

    await pixhawk.upload_mission()

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

    asyncio.run(run())