import sys
import asyncio

ibis_directory = "/home/pi/ARLISS_IBIS/IB/Library"
sys.path.append(ibis_directory)

from ibis import Ibis


fuse_pin = 3
wait_time = 10
lora_sleep_time = 3 
fuse_time = 3
land_timelimit = 10
land_judge_len = 30
health_continuous_count = 3
waypoint_lat = 39.522982899999993
waypoint_lng = -119.81465159999999
waypoint_alt = 5
mission_speed = 5
image_navigation_timeout = 6 * 60
light_threshold = 300
stored_timelimit = 10
stored_judge_time = 3
released_timelimit = 10
released_judge_time = 3
lora_power_pin = 4
deamon_pass = "/home/pi/ARLISS_IBIS/IB/log/Performance_log.txt"
is_destruct_deamon = False
use_camera = False
use_gps_config = False
use_other_param_config = False


async def run():
    
    ibis = Ibis(# pixhawk
                 fuse_pin,
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
               # light
                 light_threshold,
                 stored_timelimit,
                 stored_judge_time,
                 released_timelimit,
                 released_judge_time,
               # lora
                 lora_power_pin,
                 lora_sleep_time, 
               # deamon
                 deamon_pass,
                 is_destruct_deamon,
               # other defaults
                 use_camera,
                 use_gps_config,
                 use_other_param_config)
    
    await ibis.IBIS_MISSION()
    
    
if __name__ == "__main__":
  
  asyncio.run(run())