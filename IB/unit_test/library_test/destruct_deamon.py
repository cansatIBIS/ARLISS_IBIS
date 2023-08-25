import sys

ibis_directory = "/home/pi/ARLISS_IBIS/IB/Library"
sys.path.append(ibis_directory)

from ibis import Ibis


fuse_PIN = 3
wait_time = 60
lora_sleep_time = 3 
fuse_time = 3
land_timelimit = 8
health_continuous_count = 0
waypoint_lat = 0
waypoint_lng = 0
waypoint_alt = 0
mission_speed = 0
light_threshold = 400
stored_timelimit = 60
stored_judge_time = 10
released_timelimit = 8
released_judge_time = 5
lora_power_Pin = 4
deamon_pass = "/home/pi/ARLISS_IBIS/IB/log/Performance_log.txt"
is_destruct_deamon = True


if __name__ == "__main__":
    
    Ibis = Ibis(# pixhawk
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
               # light
                 light_threshold,
                 stored_timelimit,
                 stored_judge_time,
                 released_timelimit,
                 released_judge_time,
               # lora
                 lora_power_Pin,
               # deamon
                 deamon_pass = "/home/pi/ARLISS_IBIS/IB/log/Performance_log.txt",
                 is_destruct_deamon = True)
    
    Ibis.destruct_deamon()