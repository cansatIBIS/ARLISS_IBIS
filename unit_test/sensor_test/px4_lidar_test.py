import asyncio
from mavsdk import System

async def run():
    drone = System()
    status_test_task = asyncio .ensure_future(print_status_text(drone))
    async for distance in drone.telemetry.distance_sensor():
        for i in range(2):
             print(f"distance %d: {distance.current_distance_m}"%i)
    status_test_task.cancel()

async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return

if __name__=="__main__":
    asyncio.run(run())
