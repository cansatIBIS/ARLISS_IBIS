import asyncio
from mavsdk import System
import time
import RPi.GPIO as GPIO
from logger import logger_info
from mavsdk.mission import (MissionItem, MissionPlan)

is_landed = False
PIN = 5
is_fused = False
is_mission_finished = False
mode = None
goal = [35.797379299999996, 139.8922272]

async def run():
    drone = System()
    logger_info.info("-- Waiting for drone to be connected...")
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            logger_info.info(f"-- Connected to drone!")
            break
        
    print_alt_task = asyncio.create_task(print_alt(drone))
    land_judge_task = asyncio.create_task(land_judge(drone))
    
    await print_alt_task
    await land_judge_task

    if is_fused:
        get_log_task = asyncio.create_task(get_log(drone))
        land_task = asyncio.create_task(land(drone))

        mission_items = []
        mission_items.append(MissionItem(goal[0],
                                         goal[1],
                                         3,
                                         5,
                                         True, #止まらない
                                         float('nan'),
                                         float('nan'),
                                         MissionItem.CameraAction.NONE,
                                         float('nan'),
                                         float('nan'),
                                         float('nan'),
                                         float('nan'),
                                         float('nan')))

        mission_plan = MissionPlan(mission_items)

        await drone.mission.set_return_to_launch_after_mission(False)

        logger_info.info("-- Uploading mission")

        await drone.mission.upload_mission(mission_plan)

        logger_info.info("Waiting for drone to have a global position estimate...")

        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                logger_info.info("-- Global position estimate OK")
                break

        logger_info.info("-- Arming")
        await drone.action.arm()

        logger_info.info("-- Starting mission")
        await drone.mission.start_mission()
        
        await get_log_task
        await land_task


async def land_judge(drone):
    start_time = time.time()
    while True:
        time_now = time.time()
        if time_now-start_time < 30:
            try :
                alt_now = await(asyncio.wait_for(get_distance_alt(drone), timeout = 0.8))
            except asyncio.TimeoutError:
                continue
                
            if is_judge_alt(alt_now):
                true_dist = IQR_removal(await get_alt_list(drone, "LIDAR"))
                if len(true_dist) == 0:
                    continue
                try:
                    ave = sum(true_dist)/len(true_dist)
                except ZeroDivisionError as e:
                    logger_info.info(e)
                    continue
                
                if is_low_alt(ave):
                    for distance in true_dist:
                        if abs(ave-distance) > 0.01:
                            logger_info.info("-- Moving")
                            break
                    else:
                        true_posi = IQR_removal(await get_alt_list(drone, "POSITION"))
                        if len(true_posi) == 0:
                            continue
                        try:
                            ave = sum(true_posi)/len(true_posi)
                        except ZeroDivisionError as e:
                            logger_info.info(e)
                            continue
                        for position in true_posi:
                            if abs(ave-position) > 0.01:
                                logger_info.info("-- Moving. Lidar might have some error")
                                break
                        else:
                            is_landed = True
                        
                    if is_landed:
                        logger_info.info("-- Lidar & Position Judge")
                        break
                else:
                    logger_info.info("-- Over 1m")
        else:
            is_landed = True
            if is_landed:
                logger_info.info("-- Timer Judge")
                break
                
    
    logger_info.info("-- Land judge finish")

        
def is_low_alt(alt):
    if alt < 1:
        return True
    else:
        return False


def is_judge_alt(alt):
    if alt < 15:
        logger_info.info("-- Land judge start")
        return True
    else:
        return False
        
        
async def get_alt_list(drone, priority):
    altitude_list = []
    iter = 0
    while True:
        if priority == "LIDAR":
            try :
                distance = await asyncio.wait_for(get_distance_alt(drone), timeout = 0.8)
            except asyncio.TimeoutError:
                logger_info.info("Distance sensor might have some error")
                altitude_list =[]
                return altitude_list
            altitude_list.append(distance)
            
        elif priority == "POSITION":
            try:
                position = await asyncio.wait_for(get_position_alt(drone), timeout = 0.8)
            except asyncio.TimeoutError:
                logger_info.info("Pixhawk might have some error")
                altitude_list =[]
                return altitude_list
            altitude_list.append(position)
            
        iter += 1
        if iter >= 30:
            break
    return altitude_list
        

def IQR_removal(data):
    data.sort()
    quartile_25 = (data[7]+data[8])/2
    quartile_75 = (data[22]+data[23])/2
    IQR = quartile_75-quartile_25
    true_data = [i for i in data if quartile_25-1.5*IQR <= i <= quartile_75+1.5*IQR]
    return true_data


async def print_alt(drone):
    while True:
        try:
            position = await asyncio.wait_for(get_position_alt(drone), timeout = 0.8)
            logger_info.info("altitude:{}".format(position))
            break
        except asyncio.TimeoutError:
            logger_info.info("Pixhawk might have some error")
            pass
        await asyncio.sleep(0)
        

async def get_distance_alt(drone):
    async for distance in drone.telemetry.distance_sensor():
        return distance.current_distance_m
    

async def get_position_alt(drone):
    async for position in drone.telemetry.position():
        return position.absolute_altitude


def wait():
    logger_info.info("-- Waiting")
    time.sleep(5)
    logger_info.info("5秒経過")
    time.sleep(5)
    logger_info.info("10秒経過")
    time.sleep(5)
    logger_info.info("15秒経過")


def fusing():
    global is_fused
    try:
        logger_info.info("-- Fuse start")
        time.sleep(3)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(PIN, GPIO.OUT)

        GPIO.output(PIN, 0)
        logger_info.info("-- Fusing")

        time.sleep(5.0)
        logger_info.info("--Nichrome Wire Fused")

        GPIO.output(PIN, 1)
        is_fused = True
        time.sleep(5.0)
    
    except:
        GPIO.output(PIN, 1)

async def land(drone):
    global is_mission_finished
    while True:
        await asyncio.sleep(1)
        is_mission_finished = await drone.mission.is_mission_finished()
        if is_mission_finished:
            break
    logger_info.info("-- Landing")
    await drone.action.land()
    if str(mode) == "Land":
        logger_info.info("-- Mission Complete")

async def get_log(drone):
    global mode
    async for flight_mode in drone.telemetry.flight_mode():
        mode = flight_mode
    async for distance in drone.telemetry.distance_sensor():
        lidar = distance.current_distance_m
    async for position in drone.telemetry.position():
        lat = position.latitude_deg
        lng = position.longitude_deg
    while True:
        log_txt = (
            " mode:"
            + str(mode)
            + " lidar: "
            + str(lidar)
            + "m"
            + "latitude:"
            + str(lat)
            + "deg"
            + "longitude"
            + str(lng)
            + "deg"
            )
        logger_info.info(str(log_txt))
        await asyncio.sleep(0.5)

        if is_mission_finished:
            break

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(run())
    wait()
    fusing()