import asyncio
from mavsdk import System


async def print_gps():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break
    # for i in range(10):
        # print("a")
    async for position in drone.telemetry.position():
            print("b")
            print(position)
            break
            
asyncio.run(print_gps())