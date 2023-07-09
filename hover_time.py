import asyncio
from mavsdk import System
from logger import logger_info, logger_debug
import time

altitude = 2.5
start = None

async def run():
    global start
    drone = System()
    print("Waiting for drone to connect...")
    # await drone.connect(system_address="udp://:14540")
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    
    logger_info.info("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            logger_info.info("-- Connected to drone!")
            break
        

    arm_takeoff_task = asyncio.create_task(arm_takeoff(drone))
    print_time_task = asyncio.create_task(print_time())

    
    start = time.time()
    await arm_takeoff_task
    await print_time_task

async def arm_takeoff(drone):
    print("-- Arming")
    logger_info.info("-- Arming")
    await drone.action.arm()
    print("-- Armed")
    logger_info.info("-- Armed")
    print("-- Taking off")
    logger_info.info("-- Taking off")
    await drone.action.set_takeoff_altitude(altitude)
    await drone.action.takeoff()

    while True:
        await asyncio.sleep(1)


async def print_time():
    while True:
        now = time.time()
        print(f"{now-start}s")
        await asyncio.sleep(1)

        

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(run())