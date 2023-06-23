import numpy as np
import asyncio
from mavsdk import System
from logger import logger_info


async def run():
    drone = System()
    print("--Waiting for drone to connected...")
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            logger_info.info("-- Connected to drone!")
            break
    land_judge()


async def land_judge(drone):
    is_landed = False
    while True:
        true_distance = IQR_removal(alt_list(drone))
        num = len(true_distance)
        ave = sum(true_distance)/num
        if is_low_alt(ave):
            for i in range(num):
                if abs(ave-true_distance[i]) > 0.01:
                    break
                if i == num:
                    is_landed = True
            if is_landed:
                print("--Landed")
                break
    
        
async def is_low_alt(alt):
    if alt <= 1:
        return True
        
        
async def alt_list(drone):
    distance_list = []
    iter = 0
    async for distance in drone.telemetry.distance_sensor():
        iter += 1
        distance_list.append(distance.current_distance_m)
        if iter >= 100:
            break
    return distance_list
        

async def IQR_removal(data):
    data.sort()
    quartile_25 = (data[24]+data[25])/2
    quartile_75 = (data[74]+data[75])/2
    IQR = quartile_75-quartile_25
    center = (data[49]+data[50])/2
    true_data = [i for i in data if quartile_25+1.5*IQR< i < quartile_75+1.5*IQR]
    return true_data


if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(run())
    