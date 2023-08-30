import sys

ibis_directory = "/home/pi/ARLISS_IBIS/IB/Library"
sys.path.append(ibis_directory)

import asyncio
import numpy as np
from pixhawk import Pixhawk
from logger_lib import logger_info
from lora import Lora
from camera import Camera

# parameters---------------------
fuse_PIN = 0
wait_time = 0
fuse_time = 0
land_timelimit = 0
land_judge_len = 30
health_continuous_count = 3
waypoint_lat = 35.79695235999999
waypoint_lng = 139.8920656
waypoint_alt = 5
mission_speed = 5
lora_power_pin = 4
lora_sleep_time = 0
use_camera = True
hsv_min_1 = np.array([0,145,0])
hsv_max_1 = np.array([5,255,255])
hsv_min_2 = np.array([0,110,0])
hsv_max_2 = np.array([179,255,255])
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
                 use_camera
                 )
    
    camera = Camera(hsv_min_1,
                    hsv_max_1,
                    hsv_min_2,
                    hsv_max_2)
    
    await pixhawk.connect()

    await pixhawk.upload_mission()

    await pixhawk.health_check()

    await pixhawk.arm()

    await pixhawk.start_mission()

    await pixhawk.gather_main_coroutines()
    
    camera.take_pic()
    res = camera.detect_center()
    logger_info.info('percent={}, center={}'.format(res['percent'], res['center']))

    if res['percent'] <= 0.001:
        logger_info.info(f"Failed image navigation")
        await pixhawk.land()
    else:
        logger_info.info(f"Target detected!")
        await pixhawk.image_navigation_goto()
        await pixhawk.land()



    


if __name__ == "__main__":

    asyncio.run(run())