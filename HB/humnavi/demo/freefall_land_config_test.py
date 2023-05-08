# Version 1: without optical goal tracking
import asyncio
import json
import os
import sys
import time

import aiorun
import mavsdk
from omegaconf import OmegaConf

sys.path.append(os.getcwd())

# from humnavi.light import LightSensor
from humnavi.pixhawk import Pixhawk

async def freefall_land(location_dict: dict, conf: OmegaConf) -> None:

    # takeoff
    takeoff_altitude = 30
    await pixhawk.set_maximum_speed(10)
    maximum_speed = await pixhawk.drone.action.get_maximum_speed()
    print("maximum speed is ", maximum_speed)
    await pixhawk.hold()
    await asyncio.sleep(5)
    await pixhawk.arm()
    await asyncio.sleep(3)
    await pixhawk.set_takeoff_altitude(takeoff_altitude)
    await pixhawk.takeoff()
    await asyncio.sleep(40)
    await pixhawk.hold()

    # kill pixhawk
    # await pixhawk.kill()

    # await pixhawk.takeoff()
    # await asyncio.sleep(60)

    # free_fall
    deploy_time = time.time()
    # light.is_armopen = True
    await pixhawk.task_hold_wait()
    # await pixhawk.wait_arm_open(light)
    # print("arming in 5 seconds")
    await asyncio.sleep(5)
    # while (pixhawk.armed != True):
    #     await pixhawk.arm()
    #     await asyncio.sleep(0.01)

    arm_time = time.time()
    # print("armed after deploy ", arm_time - deploy_time, " sec.")
    while time.time() - arm_time < conf.arm_goto_s:
            await asyncio.sleep(0.01)
    print("freefall end at", pixhawk.lidar, "m")

    # goto
    target_name = conf.mission_name + "_target"
    await pixhawk.task_goto_location(
        location_dict, target_name, 30
    )
    # await pixhawk.set_maximum_speed(0.5)
    await pixhawk.task_land()

async def main(location_dict: dict, conf: OmegaConf) -> None:
    await pixhawk.connect(SYSTEM_ADDRESS)
    print("drone connected!")
    main_coroutines = [
        pixhawk.cycle_armed(),
        pixhawk.cycle_flight_mode(),
        pixhawk.cycle_odometry(),
        pixhawk.cycle_gps_info(),
        pixhawk.cycle_gps_position(),
        pixhawk.cycle_attitude(),
        pixhawk.cycle_is_landed(),
        # pixhawk.cycle_lidar(),
        pixhawk.cycle_fake_lidar(),
        pixhawk.cycle_time(),
        pixhawk.cycle_record_log(),
        freefall_land(location_dict, conf),
    ]
    await asyncio.gather(*main_coroutines)


if __name__ == "__main__":
    print("pixhawk or gazebo options: usb, pin, gazebo")
    drone = input()
    if drone == "usb":
        SYSTEM_ADDRESS = "serial:///dev/ttyACM0:115200"
    elif drone == "pin":
        SYSTEM_ADDRESS = "serial:///dev/serial0:921600"
    elif drone == "gazebo":
        SYSTEM_ADDRESS = "udp://:14540"
    with open("config/waypoints_gps.json", mode="r") as f:
        location_dict = json.load(f)
    with open("config/freefall_goto_example.yaml", mode="r") as f:
        conf = OmegaConf.load(f)
    pixhawk = Pixhawk()
    # light = LightSensor()
    print("initialized!")
    aiorun.run(main(location_dict, conf))
