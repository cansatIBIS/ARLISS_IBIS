#!/usr/bin/env python3

import asyncio
from mavsdk import System
import atexit
import csv
import datetime

absolute_altitude = 0
lat_deg_per_m = 0.000008983148616
lng_deg_per_m = 0.000008983668124
lat_list = []
lng_list = []
goal = [35.797311799999996, 139.8922149]
diff = 3
async def run():
    drone = System()
    global absolute_altitude
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

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

    await arm(drone)

    take_off_task = asyncio.ensure_future(take_off(drone))
    get_gps_task = asyncio.ensure_future(get_gps(drone))
    goto_task = asyncio.ensure_future(goto(drone))
    await take_off_task
    await get_gps_task
    await goto_task


async def arm(drone):
    print("-- Arming")
    await drone.action.arm()

async def take_off(drone):
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(1)

async def get_gps(drone):
     while True:
        async for position in drone.telemetry.position():
            lat = position.latitude_deg
            lng = position.longitude_deg
            lat_list.append(lat)
            lng_list.append(lng)
            print(f"lat:{lat}, lng:{lng}")
            break
        await asyncio.sleep(0.1)

async def goto(drone):
    global absolute_altitude, diff
    lat_diff = abs(lat_list[-1]-goal[0])
    lon_diff = abs(lng_list[-1]-goal[1])
    flying_alt = absolute_altitude + 3.0
    # goto_location() takes Absolute MSL altitude
    await drone.action.goto_location(goal[0], goal[1], flying_alt, 0)
    print(f"goto:{goal[0]}, {goal[1]}") 
    while True:
        print("go to location...")
        await asyncio.sleep(1)
        if lat_diff < diff*lat_deg_per_m and lon_diff < diff*lng_deg_per_m:
            print("reached location!")
            break
    await drone.action.land()

@atexit.register
def get_csv():
    dt_now = datetime.datetime.now()
    with open(f"/home/pi/ARLISS_IBIS/log/log_csv/goto_example {dt_now}.csv","w") as file:
        writer = csv.writer(file)
        writer.writerow(lat_list)
        writer.writerow(lng_list)

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())