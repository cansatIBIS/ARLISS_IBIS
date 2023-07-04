import asyncio
from mavsdk import System

is_judge_alt = False
is_low_alt = False
is_landed = False
is_distance_sensor_OK = True

async def run():
    drone = System()
    print("--Waiting for drone to connected...")
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
        
    print_alt_task = asyncio.create_task(print_alt(drone))
    land_judge_task = asyncio.create_task(land_judge(drone))
    
    await print_alt_task
    await land_judge_task


async def land_judge(drone):    
    while True:
        try :
            alt_now = await(asyncio.wait_for(distance_alt(drone), timeout = 0.8))
        except asyncio.TimeoutError:
            alt_now = await position_alt(drone)
            #positionの15m以下でいいのか、abs or reative
            
        judge_alt(alt_now)
            
        if judge_alt:
            true_dist = IQR_removal(await alt_list(drone))
            if is_distance_sensor_OK:
                try:
                    ave = sum(true_dist)/len(true_dist)
                except ZeroDivisionError as e:
                    print(e)
                    continue
                
            else:
                true_dist = IQR_removal(await alt_list(drone))
                try:
                    ave = sum(true_dist)/len(true_dist)
                except ZeroDivisionError as e:
                    print(e)
                    continue
                
            low_alt(ave)
            
            if is_low_alt:
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

        
def low_alt(alt):
    if alt < 1:
        is_low_alt = True
    else:
        is_low_alt = False


def judge_alt(alt):
    if alt < 15:
        print("####### land judge start #######")
        is_judge_alt = True
    else:
        is_low_alt = False
        
        
async def alt_list(drone):
    altitude_list = []
    iter = 0
    while True:
        if is_distance_sensor_OK:
            try :
                distance = await asyncio.wait_for(distance_alt(drone), timeout = 0.8)
            except asyncio.TimeoutError :
                print("Distance sensor might have some error")
                is_distance_sensor_OK = False
                return
            altitude_list.append(distance)
        else:
            position = await position_alt(drone)
            altitude_list.append(position)
        iter += 1
        if iter >= 100:
            break
    return altitude_list
        

def IQR_removal(data):
    data.sort()
    quartile_25 = (data[24]+data[25])/2
    quartile_75 = (data[74]+data[75])/2
    IQR = quartile_75-quartile_25
    true_data = [i for i in data if quartile_25-1.5*IQR <= i <= quartile_75+1.5*IQR]
    return true_data


async def print_alt(drone):
    while True:
        async for position in drone.telemetry.position():
            print("altitude:{}".format(position.absolute_altitude_m))
            break
        await asyncio.sleep(0)
        

async def distance_alt(drone):
    async for distance in drone.telemetry.distance_sensor():
        return distance.current_distance_m
    

async def position_alt(drone):
    async for position in drone.telemetry.position():
        return position.absolute_altitude


if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(run())

