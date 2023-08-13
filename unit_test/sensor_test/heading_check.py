#!/usr/bin/env python3

import asyncio
from mavsdk import System


async def run():
    # Init the drone
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

    # Start the tasks
    asyncio.ensure_future(print_heading(drone))
    asyncio.ensure_future(print_yaw(drone))

    while True:
        await asyncio.sleep(1)


async def print_heading(drone):
    async for heading in drone.telemetry.heading():
        heading_deg = heading.true_north_deg
        print(heading_deg)



async def print_yaw(drone):
    async for attitude in drone.telemetry.attitude_euler():
        yaw_deg = attitude.yaw_deg
        print(yaw_deg)


if __name__ == "__main__":
    asyncio.run(run())
