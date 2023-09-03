import sys

ibis_directory = "/home/pi/ARLISS_IBIS/IB/Library"
sys.path.append(ibis_directory)

import asyncio
from pixhawk import Pixhawk
from lora import Lora

# parameters---------------------
fuse_PIN = 0
wait_time = 0
lora_sleep_time = 0
fuse_time = 0
land_timelimit = 0
land_judge_len = 30
health_continuous_count = 1
waypoint_lat = 35.71583
waypoint_lng = 139.76472
waypoint_alt = 3
mission_speed = 5
lora_power_pin = 4
lora_sleep_time = 0
use_camera = True
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
                 lora,
                 use_camera = use_camera,
                 use_gps_config = False
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
    try:
        await pixhawk.start_mission()
    except Exception as e:
        print(e)

    

    await asyncio.gather(*main_coroutines)

if __name__ == "__main__":

    asyncio.run(run())