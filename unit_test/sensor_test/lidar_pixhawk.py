import asyncio
import pigpio
import time
from mavsdk import System


#高さ指定
hovering_hight = 5
                
                
async def run():

    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")


    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
        
    async for distance in drone.telemetry.distance_sensor():
        print(distance)

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())