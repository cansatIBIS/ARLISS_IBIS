import asyncio
import mavsdk
import logger


class Ibis:
    
    def __inti__(self):
        self.ibis = mavsdk.System()
    
    async def connect(self):
        logger.info("-- Waiting for drone to connect...")
        await self.connect(system_address="serial:///dev/ttyACM0:115200")
        async for state in self.ibis.core.connection_state():
            if state.is_connected:
                print("-- Drone connected!")
                break
        
    def arm(self):
        logger.info("-- Arming")
        self.ibis.action.arm()
        logger.info("-- Armed!")
        
