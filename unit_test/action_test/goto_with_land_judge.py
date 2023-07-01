#!/usr/bin/env python3

import asyncio
from mavsdk import System
import atexit
import csv
import datetime
from logger import logger_info, logger_debug
import time

absolute_altitude = 0
abs_alt = 0
lat_deg_per_m = 0.000008983148616
lng_deg_per_m = 0.000008983668124
lat_list = []
lng_list = []
alt_list = []
goal = [35.71673,139.764431]
diff = 3
state = None
async def run():
    logger_info.info("~goto_example_ver2.py~")
    drone = System()
    global absolute_altitude
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

    print("Waiting for drone to connect...")
    logger_info.info("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            logger_info.info("-- Connected to drone!")
            break
    
    land_judge_task = asyncio.create_task(land_judge(drone)) #タスク化しなくていいかも
    await land_judge_task
    ##################################################################################
    print("Waiting for drone to have a global position estimate...")
    logger_info.info("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            logger_info.info("-- Global position state is good enough for flying.")
            break

    print("Fetching amsl altitude at home location....")
    logger_info.info("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    await arm(drone)

    take_off_task = asyncio.ensure_future(take_off(drone))
    get_gps_task = asyncio.ensure_future(get_gps(drone))
    goto_task = asyncio.ensure_future(goto(drone))
    print_landed_state_task = asyncio.ensure_future(print_landed_state(drone))
    await take_off_task
    await get_gps_task
    await goto_task
    await print_landed_state_task

async def land_judge(drone):
    print("####### land judge start #######")
    logger_info.info("-- land judge start")
    
    is_landed = False
    while True:
        true_dist = IQR_removal(await get_alt_list(drone))
        try:
            ave = sum(true_dist)/len(true_dist)
        except ZeroDivisionError as e:
            print(e)
            continue
        
        if await is_low_alt(ave):
            for distance in true_dist:
                if abs(ave-distance) > 0.01:
                    print("--moving")
                    logger_info.info("--moving")
                    break
            else:
                is_landed = True
            if is_landed:
                print("--Landed")
                logger_info.info("--Landed")
                break
        else:
            print("--land rejected")
            logger_info.info("--land rejected")
    
    print("####### land judge finish #######")
    logger_info.info("--land judge finish")
    print("waiting 10s...")
    await asyncio.sleep(10)

        
async def is_low_alt(alt):
    return alt < 1
        
        
async def get_alt_list(drone):
    distance_list = []
    iter = 0
    async for distance in drone.telemetry.distance_sensor():
        iter += 1
        distance_list.append(distance.current_distance_m)
        await asyncio.sleep(0)
        if iter >= 100:
            break
    return distance_list
        

def IQR_removal(data):
    data.sort()
    quartile_25 = (data[24]+data[25])/2
    quartile_75 = (data[74]+data[75])/2
    IQR = quartile_75-quartile_25
    true_data = [i for i in data if quartile_25-1.5*IQR <= i <= quartile_75+1.5*IQR]
    return true_data


async def arm(drone):
    print("-- Arming")
    logger_info.info("-- Arming")
    await drone.action.arm()

async def take_off(drone):
    print("-- Taking off")
    logger_info.info("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(1)

async def get_gps(drone):
    global abs_alt
    while True:
        async for position in drone.telemetry.position():
            lat = position.latitude_deg
            lng = position.longitude_deg
            abs_alt = position.absolute_altitude_m
            lat_list.append(lat)
            lng_list.append(lng)
            alt_list.append(abs_alt)
            print(f"lat:{lat}, lng:{lng}")
            logger_info.info(f"lat:{lat}, lng:{lng}")
            break
        await asyncio.sleep(0.1)

async def goto(drone):
    global absolute_altitude, diff
    lat_diff = abs(lat_list[-1]-goal[0])
    lon_diff = abs(lng_list[-1]-goal[1])
    
    flying_alt = absolute_altitude + 3.0
    alt_diff = abs(alt_list[-1] - flying_alt)
    # goto_location() takes Absolute MSL altitude
    await drone.action.goto_location(goal[0], goal[1], flying_alt, 0)
    print(f"goto:{goal[0]}, {goal[1]}")
    logger_info.info(f"goto:{goal[0]}, {goal[1]}")
    while True:
        print("go to location...")
        logger_info.info("go to location...")
        await asyncio.sleep(1)
        if lat_diff < diff*lat_deg_per_m and lon_diff < diff*lng_deg_per_m:
            print("reached location!")
            logger_info.info("reached location!")
            break
    await drone.action.land()
    while True:
        start = time.time()
        if state == "ON_GROUND":
            print("landed")
            break
        elif time.time()-start>30:
            print("time up landed")
            break
    print(f"accuracy:lat={abs(lat_list[-1]-goal[0])*lat_deg_per_m}m, lng={abs(lng_list[-1]-goal[1])*lng_deg_per_m}m")

async def print_landed_state(drone):
    global state
    async for landed_state in drone.telemetry.landed_state():
        state = landed_state
        await asyncio.sleep(0.1)


        


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