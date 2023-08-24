from pixhawk import Pixhawk
from lora import Lora
from light import Light
import asyncio


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
        
        self.pixhawk = Pixhawk(fuse_PIN,
                          wait_time,
                          lora_sleep_time, 
                          fuse_time,
                          land_timelimit,
                          altitude,
                          latitude_deg,
                          longitude_deg,
                          max_speed,
                          lidar)
        
        self.light = Light(light_threshold,
                      stored_timelimit,
                      stored_judge_time,
                      released_timelimit,
                      released_judge_time)
        
        self.lora = Lora(lora_power_Pin)
        
        
        self.deamon_file = deamon_file
        self.deamon_log = self.deamon_file.read()
        
    async def run():
        return