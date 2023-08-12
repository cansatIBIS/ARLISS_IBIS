#!/usr/bin/env python3
# 北に20m→南に40m
import asyncio
import csv
import datetime
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.offboard import (OffboardError, PositionNedYaw)
from logger import logger_info, logger_debug

goal = [35.7927147,139.8908122]
north_m = 5
south_m = -10
lat_deg_per_m = 0.000008983148616
lng_deg_per_m = 0.000008983668124
latitude_list = []
longitude_list = []
lidar_list = []
alt_list = []
center_lat_deg = 0
center_lng_deg = 0
center_abs_alt=-2.55400013923645

async def run():
    
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

    logger_info.info("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))
    get_log_task = asyncio.ensure_future(get_log(drone))
    offboard_task = asyncio.ensure_future(offboard(drone))
    
   

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

    await termination_task
    await get_log_task
    await offboard_task


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False
    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return
        
async def get_log(drone):
    while True:
        async for flight_mode in drone.telemetry.flight_mode():
            mode = flight_mode
            break
        async for distance in drone.telemetry.distance_sensor():
            lidar = distance.current_distance_m
            break
        async for position in drone.telemetry.position():
            abs_alt = position.absolute_altitude_m
            rel_alt = position.relative_altitude_m
            break
        async for speed in drone.action.get_maxium_speed():
            max_speed = speed
            break
        async for mission_progress in drone.mission.mission_progress():
            mp_current = mission_progress.current
            mp_total = mission_progress.total
            break
        log_txt = (
            + " mode:"
            + str(mode)
            + " Mission progress:"
            + str(mp_current)
            + "/"
            + str(mp_total)
            + " lidar: "
            + str(lidar)
            + "m"
            + " abs_alt:"
            + str(abs_alt)
            + "m"
            + " rel_alt:"
            + str(rel_alt)
            + "m"
            + " max_speed:"
            +str(max_speed)
            + "m/s"
            )
        logger_info.info(str(log_txt))
        await asyncio.sleep(0.5)

async def offboard(drone):
    while True:
        await asyncio.sleep(1)
        mission_finished = await drone.mission.is_mission_finished()
        if mission_finished:
            logger_info.info("mission finished")
            break
    logger_info.info("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    logger_info.info("-- Starting offboard")

    await drone.offboard.start()


    logger_info.info("-- Go 0m North, 0m East, -2m Down \
            within local coordinate system")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(10)

    logger_info.info("-- Go 5m North, 0m East, -2m Down \
            within local coordinate system, turn to face East")
    await drone.offboard.set_position_ned(
            PositionNedYaw(5.0, 0.0, -2.0, 90.0))
    await asyncio.sleep(10)

    # logger_info.info("-- Go 5m North, 5m East, -2m Down \
    #         within local coordinate system")
    # await drone.offboard.set_position_ned(
    #         PositionNedYaw(5.0, 5.0, -2.0, 90.0))
    # await asyncio.sleep(15)

    # logger_info.info("-- Go 0m North, 5m East, 0m Down \
    #         within local coordinate system, turn to face South")
    # await drone.offboard.set_position_ned(
    #         PositionNedYaw(0.0, 5.0, 0.0, 180.0))
    # await asyncio.sleep(10)

    logger_info.info("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        logger_info.info(f"Stopping offboard mode failed \
                with error code: {error._result.result}")
    logger_info.info("landing...") 
    await drone.action.land()

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())