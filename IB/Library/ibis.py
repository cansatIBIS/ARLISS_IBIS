import pixhawk
import lora
import light

class Ibis:
    
    def __init__(self,
                 fuse_PIN,
                 wait_time,
                 lora_sleep_time, 
                 fuse_time,
                 land_timelimit,
                 altitude,
                 latitude_deg,
                 longitude_deg,
                 max_speed,
                 lidar,
                 light_threshold,
                 stored_timelimit,
                 stored_judge_time,
                 released_timelimit,
                 released_judge_time,
                 lora_power_Pin,
                 deamon_file = open("/home/pi/ARLISS_IBIS/IB/log/Performance_log.txt")):
        
        pixhawk = pixhawk(fuse_PIN,
                          wait_time,
                          lora_sleep_time, 
                          fuse_time,
                          land_timelimit,
                          altitude,
                          latitude_deg,
                          longitude_deg,
                          max_speed,
                          lidar)
        
        light = light(light_threshold,
                      stored_timelimit,
                      stored_judge_time,
                      released_timelimit,
                      released_judge_time)
        
        lora = lora(lora_power_Pin)