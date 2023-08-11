import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from logger_lib import logger_info, logger_debug
import time
import datetime
import RPi.GPIO as GPIO


class Pixhawk:
    
    def __init__(self):
        
        self.pix = System()
        
        self.PIN = 6

        self.altitude = 3.0
        
        self.flight_mode = None
        
        self.is_judge_alt = False
        self.is_low_alt = False

    async def get_flight_mode(self):
        async for flight_mode in self.pix.telemetry.flight_mode():
            self.flight_mode = flight_mode
    
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
        while True:
            asyncio.sleep(1)
            if str(self.flight_mode) == "LAND":
                logger_info.info("Landed!")
                break
        
    async def land_judge(self):
        
        start_time = time.time()
        while True:
            time_now = time.time()
            if time_now-start_time < 30:
                try :
                    alt_now = await(asyncio.wait_for(self.get_distance_alt()), timeout = 0.8)
                except asyncio.TimeoutError:
                    continue
                    
                self.is_judge_alt(alt_now)
                    
                if self.is_judge_alt:
                    true_dist = self.IQR_removal(await self.get_alt_list("LIDAR"))
                    if len(true_dist) == 0:
                        continue
                    try:
                        ave = sum(true_dist)/len(true_dist)
                    except ZeroDivisionError as e:
                        print(e)
                        continue
                        
                    self.is_low_alt(ave)
                    
                    if self.is_low_alt:
                        for distance in true_dist:
                            if abs(ave-distance) > 0.01:
                                print("-- Moving")
                                break
                        else:
                            true_posi = self.IQR_removal(await self.get_alt_list("POSITION"))
                            if len(true_posi) == 0:
                                continue
                            try:
                                ave = sum(true_posi)/len(true_posi)
                            except ZeroDivisionError as e:
                                print(e)
                                continue
                            for position in true_posi:
                                if abs(ave-position) > 0.01:
                                    print("-- Moving. Lidar might have some error")
                                    break
                            else:
                                is_landed = True
                            
                        if is_landed:
                            print("-- Lidar & Position Judge")
                            break
                    else:
                        print("-- Over 1m")
            else:
                is_landed = True
                print("-- Timer Judge")
                break
                    
        
        print("####### Land judge finish #######")

            
    def is_low_alt(self, alt):
        
        if alt < 1:
            self.is_low_alt = True
        else:
            self.is_low_alt = False


    def is_judge_alt(self, alt):
        
        if alt < 15:
            print("####### Land judge start #######")
            self.is_judge_alt = True
        else:
            self.is_judge_alt = False
            
            
    async def get_alt_list(self, priority):
        
        altitude_list = []
        iter = 0
        while True:
            if priority == "LIDAR":
                try :
                    distance = await asyncio.wait_for(self.get_distance_alt(), timeout = 0.8)
                except asyncio.TimeoutError:
                    print("Distance sensor might have some error")
                    altitude_list =[]
                    return altitude_list
                altitude_list.append(distance)
                
            elif priority == "POSITION":
                try:
                    position = await asyncio.wait_for(self.get_position_alt(), timeout = 0.8)
                except asyncio.TimeoutError:
                    print("Pixhawk might have some error")
                    altitude_list =[]
                    return altitude_list
                altitude_list.append(position)
                
            iter += 1
            if iter >= 30:
                break
        return altitude_list
            

    def IQR_removal(self, data):
        
        data.sort()
        quartile_25 = (data[7]+data[8])/2
        quartile_75 = (data[22]+data[23])/2
        IQR = quartile_75-quartile_25
        true_data = [i for i in data if quartile_25-1.5*IQR <= i <= quartile_75+1.5*IQR]
        return true_data


    async def print_alt(self):
        
        while True:
            try:
                position = await asyncio.wait_for(self.get_position_alt(), timeout = 0.8)
                print("altitude:{}".format(position))
                break
            except asyncio.TimeoutError:
                print("Pixhawk might have some error")
                pass
            await asyncio.sleep(0)
            

    async def get_distance_alt(self):
        
        async for distance in self.telemetry.distance_sensor():
            return distance.current_distance_m
        

    async def get_position_alt(self):
        
        async for position in self.telemetry.position():
            return position.absolute_altitude

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
            
    async def mission(self, waypoint):
        mission_items = []
        mission_items.append(MissionItem(waypoint[0],
                                        waypoint[1],
                                        self.altitude, # rel_alt
                                        5, # speed
                                        True, #止まらない
                                        float('nan'),
                                        45, #gimbal_yaw_deg
                                        MissionItem.CameraAction.NONE,
                                        float('nan'),
                                        float('nan'),
                                        float('nan'),
                                        float('nan'),
                                        float('nan'))) #Absolute_yaw_deg, 45にするのこっちかも

        mission_plan = MissionPlan(mission_items)

        await self.pix.mission.set_return_to_launch_after_mission(False)

        logger_info.info("-- Uploading mission")

        await self.pix.mission.upload_mission(mission_plan)

        logger_info.info("Waiting for drone to have a global position estimate...")
        
        self.health_check()

        self.arm()

        logger_info.info("-- Starting mission")
        await self.pix.mission.start_mission()
        
        # print_mission_progress_task = asyncio.ensure_future(self.print_mission_progress())
        # running_tasks = [print_mission_progress_task]
        # termination_task = asyncio.ensure_future(self.observe_is_in_air(running_tasks))
        # land_task = asyncio.ensurefuture(self.misssion_land())
        # mission_items = []
        # for i in range(len(waypoints)):
        #     mission_items.append(MissionItem(waypoints[i][0],
        #                              waypoints[i][1],
        #                              waypoints[i][2],
        #                              5,
        #                              True, #止まらない
        #                              float('nan'),
        #                              float('nan'),
        #                              MissionItem.CameraAction.NONE,
        #                              float('nan'),
        #                              float('nan'),
        #                              float('nan'),
        #                              float('nan'),
        #                              float('nan')))
        # mission_plan = MissionPlan(mission_items)
        # await self.mission.set_return_to_launch_after_mission(False)
        # logger_info.info("-- Uploading mission")
        # await self.mission.upload_mission(mission_plan)
        # self.health_check()
        # self.arm()
        # logger_info.info("-- Starting mission")
        # await self.mission.start_mission()
        # await termination_task
        # await land_task
        
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