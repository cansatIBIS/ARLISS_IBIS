import asyncio
import mavsdk
from logger import logger_info


async def cycle_is_landed(self) -> None:
        while True:
            if self.land_detect:
                if float(self.lidar) < 0.10 and float(self.lidar) > 0 and self.is_detect == False:
                    await self.land()
                    await asyncio.sleep(3)
                    await self.task_kill_forever()
                elif abs(float(self.roll_deg)) > 60 or abs(float(self.pitch_deg)) > 60:
                    if self.armed:
                        logger_info.info("hit the target!")
                        await self.task_kill_forever()
                await asyncio.sleep(0.01)
            else:
                await asyncio.sleep(0.01)
                
                
async def task_goto_land_without_detection(self, conf):
        logger_info.info("task goto land without detection has started.")
        if self.lidar < 8 and self.lidar > 0:
            self.abs_alt_target = self.absolute_altitude_m - self.lidar + conf.land_alt_m
        try:
            await self.goto_location(self.lat_target, self.lng_target, self.abs_alt_target, 0)
            await asyncio.sleep(conf.final_goto_duration_s)
        except mavsdk.action.ActionError:
            await asyncio.sleep(0.01)
        return
    
    
async def task_land(self):
        self.land_detect = True
        while True:
            if str(self.flight_mode) != "LAND":
                try:
                    await self.land()
                except mavsdk.action.ActionError:
                    logger_info.info("land ActionError")
                    await asyncio.sleep(0.1)
            else:
                break
            
            
async def task_kill_forever(self):
        self.land_detect = False
        while True:
            await self.kill()
            await asyncio.sleep(0.1)
