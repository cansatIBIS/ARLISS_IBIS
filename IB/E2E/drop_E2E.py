import asyncio
from mavsdk import System
import time
import RPi.GPIO as GPIO


async def run():
    drone = System()
    print("--Waiting for drone to connected...")
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
        
    alt_task = asyncio.create_task(get_alt(drone))
    land_judge_task = asyncio.create_task(land_judge(drone))
    
    await alt_task
    await land_judge_task


async def land_judge(drone):
    print("####### land judge start #######")
    
    is_landed = False
    while True:
        true_dist = IQR_removal(await alt_list(drone))
        try:
            ave = sum(true_dist)/len(true_dist)
        except ZeroDivisionError as e:
            print(e)
            continue
        
        if await is_low_alt(ave):
            for distance in true_dist:
                if abs(ave-distance) > 0.01:
                    print("--moving")
                    break
            else:
                is_landed = True
            if is_landed:
                print("--Landed")
                break
        else:
            print("--rejected")
    
    print("####### land judge finish #######")

        
async def is_low_alt(alt):
    return alt < 1
        
        
async def alt_list(drone):
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


async def get_alt(drone):
    while True:
        async for position in drone.telemetry.position():
            print("altitude:{}".format(position.absolute_altitude_m))
            break
        await asyncio.sleep(1)


PIN = 5

def fusing():
    try:
        print("-- Start")
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(PIN, GPIO.OUT)

        GPIO.output(PIN, 0)
        print("-- Fusing")

        time.sleep(1.2)
        print("-- Fused! Please Fly")

        GPIO.output(PIN, 1)
    
    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(run())
    fusing()

