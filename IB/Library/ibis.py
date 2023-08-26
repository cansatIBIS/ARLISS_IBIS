import asyncio

from pixhawk import Pixhawk
from lora import Lora
from light import Light
from logger_lib import logger_info


class Ibis:
    
    def __init__(self,
               # pixhawk
                 fuse_pin,
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
                 lora_power_pin,
               # deamon
                 deamon_pass = "/home/pi/ARLISS_IBIS/IB/log/Performance_log.txt",
                 is_destruct_deamon = True
                 ):
        
        self.pixhawk = Pixhawk(fuse_pin,
                               wait_time,
                               lora_sleep_time, 
                               fuse_time,
                               land_timelimit,
                               health_continuous_count,
                               waypoint_lat,
                               waypoint_lng,
                               waypoint_alt,
                               mission_speed,
                               lora_power_pin,
                               deamon_pass)
        
        self.light = Light(light_threshold,
                           stored_timelimit,
                           stored_judge_time,
                           released_timelimit,
                           released_judge_time,
                           lora_power_pin,
                           deamon_pass)
        
        self.lora = Lora(lora_power_pin)
        
        self.deamon_pass = deamon_pass
        self.deamon_file = open(self.deamon_pass)
        self.deamon_log = self.deamon_file.read()
        self.is_destruct_deamon = is_destruct_deamon
        
        logger_info.info("Ibis initialized")
        
        
    async def wait_storing_phase(self):
        
        await self.pixhawk.wait_store()
        
        
    async def judge_phase(self):
        
        await self.pixhawk.connect()
        await self.light.stored_judge()
        await self.light.released_judge()
        await self.pixhawk.land_judge()
        await self.pixhawk.fusing()
        
        
    async def flying_phase(self):
        
        await self.pixhawk.upload_mission()
        await self.pixhawk.health_check()
        await self.pixhawk.arm()
        await self.pixhawk.start_mission()
        await self.pixhawk.gather_main_coroutines()
        await self.pixhawk.land()
    
    
    async def destruct_deamon(self):
        if self.is_destruct_deamon:
            with open(self.deamon_pass, "r") as deamon:
                deamon.write("")
                logger_info.info("Destructed deamon")
            
    
    async def IBIS_MISSION(self):
        
        logger_info.info("IBIS MISSION START")
        
        self.lora.write("IBIS MISSION START")
        
        await self.wait_storing_phase()
        
        await self.judge_phase()
        
        await self.flying_phase()
        
        await self.destruct_deamon()
        
        logger_info.info("IBIS MISSION COMPLETE")
        
        self.lora.write("IBIS MISSION COMPLETE")