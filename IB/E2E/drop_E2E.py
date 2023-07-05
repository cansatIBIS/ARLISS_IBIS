import asyncio
from mavsdk import System
import time
import RPi.GPIO as GPIO
from logger_E2E import logger_info

is_landed = False
PIN = 5

# 審査会でGPS取れるなら
async def run():
    drone = System()
    logger_info.info("--Waiting for drone to connected...")
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            logger_info.info("-- Connected to drone!")
            break
        
    # alt_task = asyncio.create_task(print_alt(drone))
    # land_judge_task = asyncio.create_task(land_judge(drone))
    
    # await alt_task
    # await land_judge_task
    await land_judge(drone)


async def land_judge(drone):
    logger_info.info("####### land judge start #######")
    start_time = time.time()
    while True:
        time_now = time.time()
        if time_now-start_time < 30:
            true_dist = IQR_removal(await alt_list(drone))
            try:
                ave = sum(true_dist)/len(true_dist)
            except ZeroDivisionError as e:
                print(e)
                continue
            
            if await is_low_alt(ave):
                for distance in true_dist:
                    if abs(ave-distance) > 0.01:
                        logger_info.info("--moving")
                        break
                else:
                    is_landed = True
        else:
            is_landed = True
            
        if is_landed:
                    logger_info.info("--Landed")
                    break
    
    logger_info.info("####### land judge finish #######")

        
async def is_low_alt(alt):
    return alt < 1
        
        
async def alt_list(drone):
    distance_list = []
    iter = 0
    async for distance in drone.telemetry.distance_sensor():
        iter += 1
        logger_info.info("altitude:{}".format(distance.current_distance_m))
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


# async def print_alt(drone):
#     while True:
#         if is_landed:
#             return
#         else:
#             async for position in drone.telemetry.position():
#                 logger_info.info("altitude:{}".format(position.absolute_altitude_m))
#                 break
#             await asyncio.sleep(0)


def fusing():
    try:
        logger_info.info("-- Fuse start")
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(PIN, GPIO.OUT)

        GPIO.output(PIN, 0)
        logger_info.info("-- Fusing")

        time.sleep(1.2)
        logger_info.info("-- Fused! Please Fly")

        GPIO.output(PIN, 1)
    
    except KeyboardInterrupt:
        GPIO.output(PIN, 1)


if __name__ == "__main__":
    time.sleep(10)
    asyncio.get_event_loop().run_until_complete(run())
    time.sleep(5)
    fusing()

