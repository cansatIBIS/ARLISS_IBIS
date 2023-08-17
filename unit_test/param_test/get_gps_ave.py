#!/usr/bin/env python3

import asyncio
import numpy as np
from mavsdk import System


async def run():
    # Init the drone
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

    lat_lst = np.empty(0)
    lng_lst = np.empty(0)
    for _ in range(10):
        lat, lng = await get_gps(drone)
        print(f"lat:{lat},lng:{lng}")
        lat_lst.append(lat)
        lng_lst.append(lng)
    print(f"lat_ave:{np.average(lat_lst)},lng_ave:{np.average(lng_lst)}")
    


async def get_gps(drone):
    async for position in drone.telemetry.position():
        lat = position.latitude_deg
        lng = position.longitude_deg
        break
    return lat, lng


if __name__ == "__main__":
    # Start the main function
    asyncio.run(run())
