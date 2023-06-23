import asyncio
from mavsdk import System
from logger import logger_info, logger_debug


async def land_judge(drone):
    async for distance in drone.telemetry.distance_sensor():
        d = distance.current_distance_m
        asyncio.sleep(1)
        
        
async def alt_change(drone):
    distance_list = []
    iter = 0
    async for distance in drone.telemetry.distance_sensor():
        iter += 1
        distance_list.append(distance.current_distance_m)
        if iter >= 9:
            break


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
        

