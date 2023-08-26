import asyncio
import mavsdk
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from logger_lib import logger_info, logger_debug
from lora import Lora
import time
import datetime
import RPi.GPIO as GPIO


class Pixhawk:
    
    def __init__(self,
                 fuse_PIN,
                 wait_time,
                 lora_sleep_time, 
                 fuse_time,
                 land_timelimit,
                 health_continuous_count,
                 waypoint_lat,
                 waypoint_lng,
                 waypoint_alt,
                 mission_speed,
                 lora_power_Pin,
                 deamon_pass = "/home/pi/ARLISS_IBIS/IB/log/Performance_log.txt"):
        
        self.pix = System()
        self.lora = Lora(lora_power_Pin)
        self.lora.serial_connect()

        self.fuse_PIN = fuse_PIN
        self.wait_time = wait_time
        self.lora_sleep_time = lora_sleep_time
        self.fuse_time = fuse_time
        self.land_timelimit = land_timelimit
        self.health_continuous_count = health_continuous_count
        self.waypoint_lat = waypoint_lat
        self.waypoint_lng = waypoint_lng
        self.waypoint_alt = waypoint_alt
        self.mission_speed = mission_speed

        self.flight_mode = None
        self.mp_current = None
        self.mp_total = None
        self.max_speed = None
        self.latitude_deg = None
        self.longitude_deg = None
        self.lidar = None
        self.tasks = None
        self.deamon_pass = deamon_pass
        self.deamon_file = open(self.deamon_pass)
        self.deamon_log = self.deamon_file.read()
        self.is_landed = False 
        self.is_judge_alt = False
        self.is_low_alt = False
        

    async def get_flight_mode(self):

        async for flight_mode in self.pix.telemetry.flight_mode():
            self.flight_mode = flight_mode
            
    async def get_mission_progress(self):
        
        async for mission_progress in self.pix.mission.mission_progress():
            self.mp_current = mission_progress.current
            self.mp_total = mission_progress.total
            

    async def get_max_speed(self):

        async for speed in self.pix.action.get_maxium_speed():
            self.max_speed = speed
            

    async def get_distance_alt(self):

        async for distance in self.pix.telemetry.distance_sensor():
            return distance.current_distance_m
        
    
    async def get_lidar(self):
        
        async for distance in self.pix.telemetry.distance_sensor():
            self.lidar = distance.current_distance_m

        
    async def get_position_alt(self):

        async for position in self.pix.telemetry.position():
            return position.absolute_altitude
        
        
    async def get_position_lat_lng(self):

        async for position in self.pix.telemetry.position():
            self.latitude_deg = position.latitude_deg
            self.longitude_deg = position.longitude_deg


    async def cycle_flight_mode(self):

        while True:
            await self.get_flight_mode()
            await asyncio.sleep(0.1)
    
    async def cycle_mission_progress(self):

        while True:
            while True:
                await self.get_mission_progress()
                await asyncio.sleep(0.1)

    async def cycle_position_lat_lng(self):

        while True:
            await self.get_position_lat_lng()
            await asyncio.sleep(0.1)
    
    
    async def cycle_lidar(self):

        while True:
            await self.get_lidar()
            await asyncio.sleep(0.1)

    async def cycle_show(self):

        while True:
            log_txt = (
                " mode:"
                + str(self.flight_mode)
                + " mission progress:"
                + str(self.mp_current)
                + "/"
                + str(self.mp_total)
                + " lat:"
                + str(self.latitude_deg)
                + " lng:"
                + str(self.longitude_deg)
                + " lidar: "
                + str(self.lidar)
                + "m"
            )
            logger_info.info(str(log_txt))
            await asyncio.sleep(0.3)
            
    
    async def connect(self):

        logger_info.info("Waiting for drone to connect...")
        await self.pix.connect(system_address="serial:///dev/ttyACM0:115200")
        async for state in self.pix.core.connection_state():
            if state.is_connected:
                logger_info.info("Drone connected!")
                break
            
        
    async def arm(self):
        
        logger_info.info("Arming")
        while True:
            try:
                await self.pix.action.arm()
            except mavsdk.action.ActionError:
                logger_info.info("Arm ActionError")
                await asyncio.sleep(0.1)
            else:
                logger_info.info("Armed!")
                break    

        
    async def takeoff(self):
        
        logger_info.info("Taking off")
        await self.pix.set_takeoff_altitude(self.altitude)
        await self.pix.takeoff()
        logger_info.info("Took off!")
        
        
    async def land(self):
        
        logger_info.info("Landing")
        await self.pix.action.land()
        await asyncio.sleep(10)
        logger_info.info("Landed!")
            
    
    async def wait_store(self):
        
        if "Three minutes passed" in self.deamon_log:
            await self.lora.write("skipped store wait")
            logger_info.info("skipped store wait")
            return
        
        else:
            logger_info.info("Waiting for store")
            time.sleep(self.wait_time)
            logger_info.info("Three minutes passed")
            
            
    async def land_judge(self):
        
        if "land judge finish" in self.deamon_log:
            await self.lora.write("skipped land judge")
            logger_info.info("skipped land judge")
            return
        
        else:
            self.lora.write("land judge start")
            start_time = time.time()
            while True:
                time_now = time.time()
                if time_now-start_time < self.land_timelimit:
                    try :
                        alt_now = await asyncio.wait_for(self.get_distance_alt(), timeout = 0.8)
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
                            logger_info.info(e)
                            continue
                            
                        self.is_low_alt(ave)
                        
                        if self.is_low_alt:
                            for distance in true_dist:
                                if abs(ave-distance) > 0.01:
                                    logger_info.info("-- Moving")
                                    break
                            else:
                                true_posi = self.IQR_removal(await self.get_alt_list("POSITION"))
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
                    logger_info.info("-- Timer Judge")
                    break
                        
            await self.lora.write("land judge finish")
            logger_info.info("####### Land judge finish #######")

            
    def is_low_alt(self, alt):
        
        if alt < 1:
            self.is_low_alt = True
        else:
            self.is_low_alt = False


    def is_judge_alt(self, alt):
        
        if alt < 15:
            logger_info.info("####### Land judge start #######")
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
                    logger_info.info("Distance sensor might have some error")
                    altitude_list =[]
                    return altitude_list
                altitude_list.append(distance)
                
            elif priority == "POSITION":
                try:
                    position = await asyncio.wait_for(self.get_position_alt(), timeout = 0.8)
                except asyncio.TimeoutError:
                    logger_info.info("Pixhawk might have some error")
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
                logger_info.info("altitude:{}".format(position))
                break
            except asyncio.TimeoutError:
                logger_info.info("Pixhawk might have some error")
                pass
            await asyncio.sleep(0)


    async def fusing(self):
        
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.fuse_Pin, GPIO.OUT, initial=GPIO.HIGH)
            logger_info.info("-- Fuse start")

            GPIO.output(self.fuse_Pin, 0)
            logger_info.info("-- Fusing")

            await asyncio.sleep(self.fuse_time)
            logger_info.info("-- Fused! Please Fly")

            GPIO.output(self.fuse_Pin, 1)
        
        except:
            GPIO.output(self.fuse_Pin, 1)
            
            
    async def health_check(self):
        
        logger_info.info("Waiting for drone to have a global position estimate...")
        
        health_true_count = 0

        async for health in self.pix.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                health_true_count += 1
            else:
                health_true_count = 0

            if health_true_count >= self.health_continuous_count:
                break
        logger_info.info("Global position estimate OK")

            
    async def upload_mission(self):
        mission_items = []
        mission_items.append(MissionItem(self.waypoint_lat,
                                        self.waypoint_lng,
                                        self.waypoint_alt,
                                        self.mission_speed,
                                        False,
                                        float('nan'),
                                        float('nan'),
                                        MissionItem.CameraAction.NONE,
                                        float('nan'),
                                        float('nan'),
                                        float('nan'),
                                        float('nan'),
                                        float('nan')))

        self.mission_plan = MissionPlan(mission_items)
        await self.pix.mission.set_return_to_launch_after_mission(False)
        logger_info.info("Uploading mission")
        await self.pix.mission.upload_mission(self.mission_plan)

        
    async def start_mission(self):

        logger_info.info("Starting mission")
        await self.pix.mission.start_mission()

        
    async def print_mission_progress(self):
        
        async for mission_progress in self.pix.mission.mission_progress():
            logger_info.info(f"Mission progress: "
                f"{mission_progress.current}/"
                f"{mission_progress.total}")
            
    
    async def observe_is_in_air(self, tasks):
        
        was_in_air = False
        async for is_in_air in self.pix.telemetry.in_air():
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
            mission_finished = await self.pix.mission.is_mission_finished()
            logger_info.info(mission_finished)
            if mission_finished:
                await self.land()


    async def gather_main_coroutines(self):

        main_coroutines = [
            self.cycle_flight_mode(),
            self.cycle_position_lat_lng(),
            self.cycle_lidar(),
            self.cycle_show(),
            self.cycle_wait_mission_finished()
        ]
        self.tasks = asyncio.gather(*main_coroutines)
        await self.tasks


    async def clear_mission(self):

        logger_info.info("Clearing mission...")
        await self.pix.mission.clear_mission()
        logger_info.info("Cleared mission")


    async def cycle_wait_mission_finished(self):
        while True:
            await asyncio.sleep(1)
            mission_finished = await self.pix.mission.is_mission_finished()
            if mission_finished:
                logger_info.info("Mission finished")
                break
        self.tasks.cancel()


    async def goto_location(self):

        logger_info.info("Setting goto_location...")
        abs_alt = await self.get_position_alt()
        logger_info.info(f"abs_alt:{abs_alt}")
        await self.pix.action.goto_location(self.waypoint_lat, self.waypoint_lng, abs_alt, 0)
        logger_info.info("Going to location...")
        await asyncio.sleep(20)