import numpy as np
import scipy.stats as stats
import asyncio
from mavsdk import System
from logger import logger_info, logger_debug


async def land_judge(drone):
    is_landed = True
    async for distance in drone.telemetry.distance_sensor():
        if low_alt_judge(distance.current_distance_m):
            true_distance = smirnov_grubbs(alt_list(drone))[0]
            num = len(true_distance)
            ave = sum(true_distance)/num
            for i in range(num):
                if abs(ave-true_distance[i]) > 0.01:
                    is_landed = False
            if is_landed:
                print("--Landed")
        
        
async def low_alt_judge(alt):
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
        

async def smirnov_grubbs(data, alpha):
	x, o = list(data), []
	while True:
		n = len(x)
		t = stats.t.isf(q=(alpha / n) / 2, df=n - 2)
		tau = (n - 1) * t / np.sqrt(n * (n - 2) + n * t * t)
		i_min, i_max = np.argmin(x), np.argmax(x)
		myu, std = np.mean(x), np.std(x, ddof=1)
		i_far = i_max if np.abs(x[i_max] - myu) > np.abs(x[i_min] - myu) else i_min
		tau_far = np.abs((x[i_far] - myu) / std)
		if tau_far < tau: break
		o.append(x.pop(i_far))
	return (np.array(x), np.array(o))


if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(run())


# altitude = 2.5

# async def run():
#     drone = System()
#     logger_info.info("Waiting for drone to connect...")
#     async for state in drone.core.connection_state():
#         if state.is_connected:
#             print(f"-- Connected to drone!")
#             logger_info.info("-- Connected to drone!")
#             break
        
#     print_altitude_task = asyncio.create_task(print_altitude(drone))
#     print_flight_mode_task = asyncio.create_task(print_flight_mode(drone))
#     arm_takeoff_task = asyncio.create_task(arm_takeoff_land(drone))
#     land_judge_task = asyncio.create_task(land_judge(drone))
#     await print_altitude_task
#     await print_flight_mode_task
#     await arm_takeoff_task
#     await land_judge_task
    

# async def arm_takeoff_land(drone):
#     print("-- Arming")
#     logger_info.info("-- Arming")
#     await drone.action.arm()
#     print("-- Armed")
#     logger_info.info("-- Armed")
#     print("-- Taking off")
#     logger_info.info("-- Taking off")
#     await drone.action.set_takeoff_altitude(altitude)
#     await drone.action.takeoff()

#     await asyncio.sleep(15)

#     print("-- Landing")
#     logger_info.info("-- Landing")
#     await drone.action.land()


# async def print_altitude(drone):
#     previous_altitude = 0.0
    
#     async for distance in drone.telemetry.distance_sensor():
#         altitude_now = distance.current_distance_m
#         print("difference : {}".format(altitude_now - previous_altitude))
#         if abs(previous_altitude - altitude_now) >= 0.1:
#             previous_altitude = altitude_now
#             print(f"Altitude: {altitude_now}")

#             logger_info.info(f"mode:{mode} lidar:{altitude_now}m")


# async def print_flight_mode(drone):
#     global mode
#     async for flight_mode in drone.telemetry.flight_mode():
#         mode = flight_mode
        

