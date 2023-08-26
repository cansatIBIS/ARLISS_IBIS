import sys

ibis_directory = "/home/pi/ARLISS_IBIS/IB/Library"
sys.path.append(ibis_directory)

import asyncio
from pixhawk import Pixhawk

# parameters---------------------
fuse_PIN = 3
wait_time = 0
lora_sleep_time = 3
fuse_time = 3
land_timelimit = 60
health_continuous_count = 10
waypoint_lat = 40.19373
waypoint_lng = 140.05923
waypoint_alt = 5
mission_speed = 5
#--------------------------------

async def run():

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
                 )
    
    await pixhawk.landjudge_and_sendgps()
    

if __name__ == "__main__":

    asyncio.run(run())