import sys

ibis_directory = "/home/pi/ARLISS_IBIS/IB/Library"
sys.path.append(ibis_directory)

import asyncio
from pixhawk import Pixhawk
from lora import Lora

# parameters---------------------
lora_power_pin = 4
lora_sleep_time = 1
fuse_PIN = 3
wait_time = 10
fuse_time = 3
land_timelimit = 30
land_judge_len = 20
health_continuous_count = 10
waypoint_lat = 40.19373
waypoint_lng = 140.05923
waypoint_alt = 5
mission_speed = 5
image_navigation_timeout = 0
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
                 image_navigation_timeout,
                 lora
                 )
    
    # print("Take care that raspi uses pin power suply to LED")
    await pixhawk.connect()
    lora.serial_connect()
    await pixhawk.wait_store()
    await pixhawk.landjudge_and_sendgps()
    pixhawk.fuse()
    # pixhawk.LED()


if __name__ == "__main__":

    asyncio.run(run())