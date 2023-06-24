import asyncio
import mavsdk
import logger


class Pixhawk:
    
    def __init__(self):
        self.pix = mavsdk.System()
    
    async def connect(self):
        logger.info("-- Waiting for drone to connect...")
        await self.pix.connect(system_address="serial:///dev/ttyACM0:115200")
        async for state in self.pix.core.connection_state():
            if state.is_connected:
                logger.info("-- Drone connected!")
                break
        
    async def arm(self):
        logger.info("-- Arming")
        await self.pix.action.arm()
        logger.info("-- Armed!")
        
    async def takeoff(self, altitude):
        logger.info("Taking off")
        await self.pix.set_takeoff_altitude(altitude)
        await self.pix.takeoff()
        logger.info("Took off!")
        
    async def land(self):
        logger.info("Landing")
        await self.pix.land()
        logger.info("Landed!")
    
