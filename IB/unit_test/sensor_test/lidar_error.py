import timeout_decorator
from mavsdk import System
import asyncio

            
@timeout_decorator.timeout(5, timeout_exception = TimeoutError)
async def get_alt(drone):
    while True:
        async for position in drone.telemetry.position():
            print("altitude:{}".format(position.absolute_altitude_m))
            break
        await asyncio.sleep(0)
        
async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    await get_alt(drone)
    
if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(run())