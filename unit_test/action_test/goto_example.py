#!/usr/bin/env python3

import asyncio
from mavsdk import System

gazebo_lat = 47.3977506
gazebo_lng = 8.5456074
per_m = 0.000009009
async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(1)
    # To fly drone  3m above the ground plane
    flying_alt = absolute_altitude + 3.0
    # goto_location() takes Absolute MSL altitude
    await drone.action.goto_location(gazebo_lat * 5 * per_m, gazebo_lng, flying_alt, 0)

    while True:
        print("Staying connected, press Ctrl-C to exit")
        await asyncio.sleep(1)
    await asyncio.sleep(10)
    await drone.action.land()
    print("-- Landing")






if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())