import asyncio
import mavsdk
from logger import logger_info, logger_debug
import time
import RPi.GPIO as GPIO


class Pixhawk:
    
    def __init__(self):
        self.pix = mavsdk.System()
        self.PIN = 6
        self.altitude = 3.0
    
    async def connect(self):
        logger_info.info("-- Waiting for drone to connect...")
        await self.pix.connect(system_address="serial:///dev/ttyACM0:115200")
        async for state in self.pix.core.connection_state():
            if state.is_connected:
                logger_info.info("-- Drone connected!")
                break
        
    async def arm(self):
        logger_info.info("-- Arming")
        await self.pix.action.arm()
        logger_info.info("-- Armed!")
        
    async def takeoff(self):
        logger_info.info("Taking off")
        await self.pix.set_takeoff_altitude(self.altitude)
        await self.pix.takeoff()
        logger_info.info("Took off!")
        
    async def land(self):
        logger_info.info("Landing")
        await self.pix.land()
        logger_info.info("Landed!")
        
    async def is_low_alt(self, alt):
        return alt < 1
            
    async def alt_list(self):
        distance_list = []
        iter = 0
        async for distance in self.pix.telemetry.distance_sensor():
            iter += 1
            distance_list.append(distance.current_distance_m)
            await asyncio.sleep(0)
            if iter >= 100:
                break
        return distance_list

    def IQR_removal(self, data):
        data.sort()
        quartile_25 = (data[24]+data[25])/2
        quartile_75 = (data[74]+data[75])/2
        IQR = quartile_75-quartile_25
        true_data = [i for i in data if quartile_25-1.5*IQR <= i <= quartile_75+1.5*IQR]
        return true_data

    async def get_alt(self):
        while True:
            async for position in self.pix.telemetry.position():
                print("altitude:{}".format(position.absolute_altitude_m))
                break
            await asyncio.sleep(1)

    async def land_judge(self):
        print("####### land judge start #######")
        
        is_landed = False
        while True:
            true_dist = self.IQR_removal(await self.alt_list(self))
            try:
                ave = sum(true_dist)/len(true_dist)
            except ZeroDivisionError as e:
                print(e)
                continue
            
            if await self.is_low_alt(self, ave):
                for distance in true_dist:
                    if abs(ave-distance) > 0.01:
                        print("--moving")
                        break
                else:
                    is_landed = True
                if is_landed:
                    print("--Landed")
                    break
            else:
                print("--rejected")
        
        print("####### land judge finish #######")

    def fusing(self):
        try:
            print("-- Start")
            GPIO.cleanup()
            GPIO.setmode(GPIO.BCM)

            GPIO.setup(self.PIN, GPIO.OUT)

            GPIO.output(self.PIN, 0)
            print("-- Fusing")

            time.sleep(1.0)
            print("-- Fused! Ready to Fly")

            GPIO.output(self.PIN, 0)
            
            GPIO.cleanup()
            
        
        except KeyboardInterrupt:
            GPIO.cleanup()
            
    async def health_check(self):
        print("Waiting for drone to have a global position estimate...")
        logger_info.info("Waiting for drone to have a global position estimate...")
        
        async for health in self.pix.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                logger_info.info("-- Global position estimate OK")
                break