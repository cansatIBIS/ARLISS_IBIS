import asyncio
from mavsdk import System

drone = System()

async def print_gps():
    for i in range(10):
            async for position in drone.telemetry.position():
                print(position.latitude_deg)
                print(position.longitude_deg)
                break
            
asyncio.run(print_gps())