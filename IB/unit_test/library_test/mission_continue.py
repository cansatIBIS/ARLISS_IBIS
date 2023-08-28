import sys

ibis_directory = "/home/pi/ARLISS_IBIS/IB/Library"
sys.path.append(ibis_directory)

import asyncio
from pixhawk import Pixhawk
from logger_lib import logger_info
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from lora import Lora

# parameters---------------------
fuse_PIN = 0
wait_time = 0
lora_sleep_time = 0
fuse_time = 0
land_timelimit = 0
land_judge_len = 30
health_continuous_count = 10
waypoint_lat = 35.792622699999995
waypoint_lng = 139.8909535
waypoint_alt = 5
mission_speed = 5
lora_power_pin = 4
lora_sleep_time = 0
#--------------------------------

async def run():
    
    lora = Lora(
        lora_power_pin,
        lora_sleep_time
        )

    pixhawk = Pixhawk(
                 fuse_PIN,
                 wait_time,
                 fuse_time,
                 land_timelimit,
                 land_judge_len,
                 health_continuous_count,
                 waypoint_lat,
                 waypoint_lng,
                 waypoint_alt,
                 mission_speed,
                 lora
                 )
    
    await pixhawk.connect()

    await pixhawk.upload_mission()

    await pixhawk.health_check()

    await pixhawk.arm()

    await pixhawk.start_mission()

    await pixhawk.gather_main_coroutines()
    
    await pixhawk.clear_mission()

    await pixhawk.upload_mission()

    await pixhawk.start_mission()

    pixhawk.tasks_cancel_ng()

    await pixhawk.gather_main_coroutines()
    
    await pixhawk.clear_mission()

    await pixhawk.land()

    
if __name__ == "__main__":

    asyncio.run(run())