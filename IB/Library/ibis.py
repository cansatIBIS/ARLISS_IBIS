from pixhawk import Pixhawk
from lora import Lora
from light import Light
from logger_lib import logger_info


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
                 
                 deamon_pass = "/home/pi/ARLISS_IBIS/IB/log/Performance_log.txt",
                 is_destruct_deamon = True
                 ):
        
        self.pixhawk = Pixhawk(fuse_PIN,
                               wait_time,
                               lora_sleep_time, 
                               fuse_time,
                               land_timelimit,
                               altitude,
                               latitude_deg,
                               longitude_deg,
                               max_speed,
                               lidar,
                               deamon_pass)
        
        self.light = Light(light_threshold,
                           stored_timelimit,
                           stored_judge_time,
                           released_timelimit,
                           released_judge_time,
                           deamon_pass)
        
        self.lora = Lora(lora_power_Pin)
        
        
        self.deamon_pass = deamon_pass
        self.deamon_file = open(self.deamon_pass)
        self.deamon_log = self.deamon_file.read()
        self.is_destruct_deamon = is_destruct_deamon
        
        
    async def wait_storing_phase(self):
        
        await self.pixhawk.wait_store()
        
        
    async def judge_phase(self):
        
        await self.pixhawk.connect()
        await self.light.stored_judge()
        await self.light.released_judge()
        await self.pixhawk.land_judge()
        await self.pixhawk.fusing()
        
        
    async def fling_phase():
        
        return
    
    
    async def destruct_deamon(self):
        if self.is_destruct_deamon:
            with open(self.deamon_pass, "r") as deamon:
                deamon.write("")
                logger_info.info("Destructed deamon")
            
    
    async def run(self):
        
        await self.wait_storing_phase()
        
        await self.judge_phase()
        
        await self.fling_phase()
        
        await self.destruct_deamon()
        
        logger_info.info("Mission Complete")