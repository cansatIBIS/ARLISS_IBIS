#!/usr/bin/env python3

import asyncio

from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
import time #時間制御用モジュール
import spidev #SPI通信のモジュール

 


async def run():

    # Init the drone
    drone = System()
    # await drone.connect(system_address="udp://:14540")
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break


   # print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))

    print("waiting for pixhawk to hold")
    flag = False
    while True:
       if flag==True:
           break
       async for flight_mode in drone.telemetry.flight_mode():
           if str(flight_mode) == "HOLD":
               print("hold確認")
               flag=True
               break
           else:
               try:
                   await drone.action.hold()
               except Exception as e:
                   print(e)
                   drone = System()
                   await drone.connect(system_address="serial:///dev/ttyACM0:115200")
                   print("Waiting for drone to connect...")
                   async for state in drone.core.connection_state():

                        if state.is_connected:
                            
                            print(f"-- Connected to drone!")
                            break
                   #mission_items = []
                   #mission_items.append(MissionItem(latitude_end,
                   #                                 longitude_end,
                   #                                 altitude_end_hover,
                   #                                 0,
                   #                                 False,
                   #                                 float('nan'),
                   #                                 float('nan'),
                   #                                 MissionItem.CameraAction.NONE,
                   #                                 float('nan'),
                   #                                 float('nan'),
                   #                                 float('nan'),
                   #                                 float('nan'),
                   #                                 float('nan')))

                   #mission_items.append(MissionItem(latitude_end,
                   #                                 longitude_end,
                   #                                 0.5,
                   #                                 0,
                   #                                 False,
                   #                                 float('nan'),
                   #                                 float('nan'),
                   #                                 MissionItem.CameraAction.NONE,
                   #                                 float('nan'),
                   #                                 float('nan'),
                   #                                 float('nan'),
                   #                                 float('nan'),
                   #                                 float('nan')))

                   #mission_plan = MissionPlan(mission_items)

                   #await drone.mission.set_return_to_launch_after_mission(False)

                   #print("-- Uploading mission")
                   #await drone.mission.upload_mission(mission_plan)
                   await asyncio.sleep(0.5)
                   break
    # print("waiting for pixhawk to hold")
    # flag = False
    # while True:
    #    if flag==True:
    #        break
    #    async for flight_mode in drone.telemetry.flight_mode():
    #        if str(flight_mode) == "HOLD":
    #            print("hold確認")
    #            flag=True
    #            break
    #        else:
    #            try:
    #                await drone.action.hold()
    #            except Exception as e:
    #                print(e)
    #    await asyncio.sleep(0.5)



    

    print("-- Arming")
    #await drone.action.arm()

    
        

    


async def print_flight_mode(drone):
    """ Prints the flight mode when it changes """

    previous_flight_mode = None

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode != previous_flight_mode:
            previous_flight_mode = flight_mode
            print(f"Flight mode: {flight_mode}")



# async def observe_is_in_air(drone, running_tasks):
#     """ Monitors whether the drone is flying or not and
#     returns after landing """

#     was_in_air = False

    # async for is_in_air in drone.telemetry.in_air():
    #     if is_in_air:
    #         was_in_air = is_in_air

    #     if was_in_air and not is_in_air:
    #         for task in running_tasks:
    #             task.cancel()
    #             try:
    #                 await task
    #             except asyncio.CancelledError:
    #                 pass
    #         await asyncio.get_event_loop().shutdown_asyncgens()

    #         return


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
