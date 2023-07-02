import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from logger import logger_info, logger_debug
import time
import datetime
import RPi.GPIO as GPIO


class Pixhawk:
    
    def __init__(self):
        self.pix = System()
        self.PIN = 6
        self.altitude = 3.0
        self.north_m = 10
        self.south_m = -15
        self.lat_deg_per_m = 0.000008983148616
        self.lng_deg_per_m = 0.000008983668124
        self.center_lat_deg = 0
        self.center_lng_deg = 0
        self.center_abs_alt=-2.55400013923645
    
    async def connect(self):
        logger_info.info("-- Waiting for drone to connect...")
        await self.pix.connect(system_address="serial:///dev/ttyACM0:115200")
        async for state in self.pix.core.connection_state():
            if state.is_connected:
                logger_info.info("-- Drone connected!")
                break
        
    async def arm(self):
        logger_info.info("-- Arming")
        await self.pix.action.arm()
        logger_info.info("-- Armed!")
        
    async def takeoff(self):
        logger_info.info("Taking off")
        await self.pix.set_takeoff_altitude(self.altitude)
        await self.pix.takeoff()
        logger_info.info("Took off!")
        
    async def land(self):
        logger_info.info("Landing")
        await self.pix.land()
        logger_info.info("Landed!")
        
    async def is_low_alt(self, alt):
        return alt < 1
            
    async def get_alt_list(self):
        distance_list = []
        iter = 0
        async for distance in self.pix.telemetry.distance_sensor():
            iter += 1
            distance_list.append(distance.current_distance_m)
            await asyncio.sleep(0)
            if iter >= 100:
                break
        return distance_list

    def IQR_removal(self, data):
        data.sort()
        quartile_25 = (data[24]+data[25])/2
        quartile_75 = (data[74]+data[75])/2
        IQR = quartile_75-quartile_25
        true_data = [i for i in data if quartile_25-1.5*IQR <= i <= quartile_75+1.5*IQR]
        return true_data

    async def get_alt(self):
        while True:
            async for position in self.pix.telemetry.position():
                print("altitude:{}".format(position.absolute_altitude_m))
                break
            await asyncio.sleep(1)

    async def land_judge(self):
        print("####### land judge start #######")
        
        is_landed = False
        while True:
            true_dist = self.IQR_removal(await self.get_alt_list(self))
            try:
                ave = sum(true_dist)/len(true_dist)
            except ZeroDivisionError as e:
                print(e)
                continue
            
            if await self.is_low_alt(self, ave):
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

    def fusing(self):
        try:
            print("-- Start")
            GPIO.cleanup()
            GPIO.setmode(GPIO.BCM)

            GPIO.setup(self.PIN, GPIO.OUT)

            GPIO.output(self.PIN, 0)
            print("-- Fusing")

            time.sleep(1.0)
            print("-- Fused! Ready to Fly")

            GPIO.output(self.PIN, 0)
            
            GPIO.cleanup()
            
        
        except KeyboardInterrupt:
            GPIO.cleanup()
            
    async def health_check(self):
        logger_info.info("Waiting for drone to have a global position estimate...")
        
        async for health in self.pix.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                logger_info.info("-- Global position estimate OK")
                break
            
    async def mission(self, waypoints):
        print_mission_progress_task = asyncio.ensure_future(self.print_mission_progress())
        running_tasks = [print_mission_progress_task]
        termination_task = asyncio.ensure_future(self.observe_is_in_air(running_tasks))
        land_task = asyncio.ensurefuture(self.misssion_land())
        mission_items = []
        for i in range(len(waypoints)):
            mission_items.append(MissionItem(waypoints[i][0],
                                     waypoints[i][1],
                                     waypoints[i][2],
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
        await self.mission.set_return_to_launch_after_mission(False)
        logger_info.info("-- Uploading mission")
        await self.mission.upload_mission(mission_plan)
        self.health_check()
        self.arm()
        logger_info.info("-- Starting mission")
        await self.mission.start_mission()
        await termination_task
        await land_task
        
    async def print_mission_progress(self):
        async for mission_progress in self.mission.mission_progress():
            print(f"Mission progress: "
                f"{mission_progress.current}/"
                f"{mission_progress.total}")
    
    async def observe_is_in_air(self, tasks):
        was_in_air = False
        async for is_in_air in self.telemetry.in_air():
            if is_in_air:
                was_in_air = is_in_air

            if was_in_air and not is_in_air:
                for task in tasks:
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        pass
                await asyncio.get_event_loop().shutdown_asyncgens()

                return
        
    async def mission_land(self):
        while True:
            await asyncio.sleep(1)
            mission_finished = await self.mission.is_mission_finished()
            print(mission_finished)
            if mission_finished:
                self.land()