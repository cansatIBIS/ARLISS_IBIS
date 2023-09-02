import asyncio
import mavsdk
import json
import numpy as np
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.offboard import (OffboardError, PositionNedYaw)
from logger_lib import logger_info
from camera import Camera
import time
import datetime
import RPi.GPIO as GPIO


class Pixhawk:
    
    def __init__(self,
                 fuse_pin,
                 wait_time,
                 fuse_time,
                 land_timelimit,
                 land_judge_len,
                 health_continuous_count,
                 waypoint_lat,
                 waypoint_lng,
                 waypoint_alt,
                 mission_speed,
                 lora,
                 deamon_pass = "/home/pi/ARLISS_IBIS/IB/log/Performance_log.txt",
                 use_camera = False,
                 use_gps_config = False,
                 use_other_param_config = False):
        
        self.pix = System()
        if use_camera:
            self.camera = Camera()
        self.lora = lora
        
        if use_gps_config:
            JSON_PASS_gps = "/home/pi/ARLISS_IBIS/IB/config/matsudo_config/GPS_matsudo_config.json"
            f = open(JSON_PASS_gps , "r")
            waypoint = json.load(f)
            self.waypoint_lat = waypoint["waypoint_lat"]
            self.waypoint_lng = waypoint["waypoint_lng"]
            f.close()
        else:
            self.waypoint_lat = waypoint_lat
            self.waypoint_lng = waypoint_lng
            
    
        if use_other_param_config:
            JSON_PASS_other_param = "/home/pi/ARLISS_IBIS/IB/config/matsudo_config/other_param_matsudo_config.json"
            f = open(JSON_PASS_other_param , "r")
            other_param = json.load(f)
            self.fuse_pin = other_param["fuse_pin"]
            self.wait_time = other_param["wait_time"]
            self.fuse_time = other_param["fuse_time"]
            self.land_timelimit = other_param["land_timelimit"]
            self.land_judge_len = other_param["land_judge_len"]
            self.health_continuous_count = other_param["health_continuous_count"]
            self.mission_speed = other_param["mission_speed"]
            self.waypoint_alt = other_param["waypoint_alt"]
            f.close()
        else:
            self.fuse_pin = fuse_pin
            self.wait_time = wait_time
            self.fuse_time = fuse_time
            self.land_timelimit = land_timelimit
            self.land_judge_len = land_judge_len
            self.health_continuous_count = health_continuous_count
            self.mission_speed = mission_speed
            self.waypoint_alt = waypoint_alt
        

        self.flight_mode = None
        self.mp_current = None
        self.mp_total = None
        self.max_speed = None
        self.latitude_deg = None
        self.longitude_deg = None
        self.voltage_v = None
        self.remaining_percent = None
        self.lidar = None
        self.image_res = None
        self.east_m = None
        self.north_m = None
        self.is_in_air =  None
        self.pitch_deg = None
        self.roll_deg = None
        
        self.deamon_pass = deamon_pass
        self.deamon_file = open(self.deamon_pass)
        self.deamon_log = ""
        
        self.is_tasks_cancel_ok = False
        self.is_landed = False 
        self.is_judge_alt = False
        self.is_low_alt = False
        
        logger_info.info("Pixhawk initialized")
        

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
            return position.absolute_altitude_m
        
        
    async def get_position_lat_lng(self):

        async for position in self.pix.telemetry.position():
            self.latitude_deg = position.latitude_deg
            self.longitude_deg = position.longitude_deg


    async def get_battery(self):

        async for battery in self.pix.telemetry.battery():
            self.voltage_v = battery.voltage_v
            self.remaining_percent = battery.remaining_percent

    
    async def get_in_air(self):

        async for is_in_air in self.pix.telemetry.in_air():
            self.is_in_air = is_in_air


    async def get_pitch_roll(self):
        
        async for angle in self.drone.telemetry.attitude_euler():
            self.pitch_deg = angle.pitch_deg
            self.roll_deg = angle.roll_deg


    async def return_in_air(self):

        async for is_in_air in self.pix.telemetry.in_air():
            return is_in_air


    async def cycle_flight_mode(self):

        try:
            while True:
                await self.get_flight_mode()
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass
        
    
    async def cycle_mission_progress(self):

        try:
            while True:
                await self.get_mission_progress()
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass


    async def cycle_position_lat_lng(self):

        try:
            while True:
                await self.get_position_lat_lng()
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass
    

    async def cycle_battery(self):

        try:
            while True:
                await self.get_battery()
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass

    
    async def cycle_lidar(self):

        try:
            while True:
                await self.get_lidar()
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass


    async def cycle_pitch_roll(self):

        try:
            while True:
                await self.get_pitch_roll()
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass


    async def cycle_is_in_air(self):

        try:
            while True:
                await self.get_in_air()
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass


    async def cycle_show(self):

        try:
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
                    + " lidar:"
                    + str(self.lidar)
                    + "m"
                )
                logger_info.info(str(log_txt))
                await asyncio.sleep(0.3)
        except asyncio.CancelledError:
            pass
            
    
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
                logger_info.info("Arming denied")
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
        while True:
            await asyncio.sleep(1)
            is_in_air = await self.return_in_air()
            logger_info.info(f"is_in_air:{is_in_air}")
            if not is_in_air:
                break
        logger_info.info("Landed!")
            
    
    async def wait_store(self):
        
        if "{} seconds passed".format(self.wait_time) in self.deamon_log:
            await self.lora.write("skipped store wait")
            logger_info.info("skipped store wait")
            return
        
        else:
            start_time = time.time()
            logger_info.info("Waiting for store")
            while True:
                time_now = time.time()
                time_passed = int(time_now-start_time)
                if time_passed < self.wait_time:
                    logger_info.info("{} seconds passed".format(time_passed)//1)
                else:
                    break
            logger_info.info("{} seconds passed. Wait phase finished".format(self.wait_time))
            
            
    async def land_judge(self):
        
        if "land judge finish" in self.deamon_log:
            logger_info.info("skipped land judge")
            return
        
        else:
            logger_info.info("-------------------- Land judge start --------------------")
            await self.lora.write("land judge start")
            start_time = time.time()
            while True:
        
                time_now = time.time()
                await asyncio.sleep(0)
                time_passed = int((time_now-start_time)//1)
                logger_info.info("Time passed:{}".format(time_passed))
                
                if time_passed < self.land_timelimit:
                    if self.is_judge_alt:
                        true_dist = self.IQR_removal(await self.get_alt_list("LIDAR"))
                        if len(true_dist) == 0:
                            true_posi = self.IQR_removal(await self.get_alt_list("POSITION"))
                            if len(true_posi) == 0:
                                continue
                            try:
                                ave = sum(true_posi)/len(true_posi)
                            except ZeroDivisionError as e:
                                logger_info.info(e)
                                continue
                            for position in true_posi:
                                if abs(ave-position) > 0.1:
                                    logger_info.info("-- Moving")
                                    continue
                            else:
                                self.is_landed = True
                                
                            if self.is_landed:
                                logger_info.info("-- Position Judge")
                                break
                        else:
                            try:
                                ave = sum(true_dist)/len(true_dist)
                            except ZeroDivisionError as e:
                                logger_info.info(e)
                                continue
                        
                        if self.is_low_alt:
                            for distance in true_dist:
                                if abs(ave-distance) > 0.01:
                                    logger_info.info("-- Moving")
                                    break
                            else:
                                self.is_landed = True
                                
                            if self.is_landed:
                                logger_info.info("-- Lidar Judge")
                                break
                        else:
                            self.change_low_alt(ave)
                            if self.is_low_alt:
                                continue
                            else:
                                logger_info.info("-- Over 1m")
                    else:
                        try :
                            alt_now = await(asyncio.wait_for(self.get_distance_alt(), timeout = 0.8))
                            self.change_judge_alt(alt_now)
                        except asyncio.TimeoutError:
                            logger_info.info("Too high or distance sensor might have some error")
                            continue
                else:
                    self.is_landed = True
                    if self.is_landed:
                        logger_info.info("-- Timer Judge")
                        break
                        
            logger_info.info("-------------------- Land judge finish --------------------")
    
    
    async def landjudge_and_sendgps(self):
        
        coroutines = [
            self.land_judge(),
            self.send_gps()
        ]
        self.judge_tasks = asyncio.gather(*coroutines)
        await self.judge_tasks
        
        
    async def send_gps(self):
        
        while True:
            
            if self.is_landed:
                break
            if self.is_judge_alt:
                break
            else:
                lat_deg, lng_deg, alt = await self.get_gps()
                self.lat = "lat:" + str(lat_deg)
                self.lng = "lng:" + str(lng_deg)
                self.alt = "alt:" + str(alt)
                await self.lora.write(self.lat)
                logger_info.info(self.lat)
                await asyncio.sleep(0)
                await self.lora.write(self.lng)
                logger_info.info(self.lng)
                await asyncio.sleep(0)
                await self.lora.write(alt)
                logger_info.info(self.alt)
                await asyncio.sleep(0)
            
            
    async def get_gps(self):
    
        lat, lng, alt = "0", "0", "0"
        try:
            await asyncio.wait_for(self.gps(), timeout=0.8)
        except asyncio.TimeoutError:
            logger_info.info("Can't catch GPS")
            lat = "error"
            lng = "error"
            alt = "error"
        return lat, lng, alt
            
            
    async def gps(self):
        
        async for position in self.pix.telemetry.position():
                logger_info.info(position)
                self.lat = str(position.latitude_deg)
                self.lng = str(position.longitude_deg)
                self.alt = str(position.relative_altitude_m)
                break

            
    def change_low_alt(self, alt):
        
        if alt < 1:
            self.is_low_alt = True
            logger_info.info("-- Under 1m")
        else:
            self.is_low_alt = False


    def change_judge_alt(self, alt):
        
        if alt < 15:
            self.is_judge_alt = True
            logger_info.info("-- Under 15m")
        else:
            self.is_judge_alt = False
            
            
    async def get_alt_list(self, priority):
        
        altitude_list = []
        pre_time = 0
        for _ in range(self.land_judge_len):
            if priority == "LIDAR":
                try :
                    distance = await asyncio.wait_for(self.get_distance_alt(), timeout = 0.8)
                except asyncio.TimeoutError:
                    logger_info.info("Too high or distance sensor might have some error")
                    altitude_list =[]
                    return altitude_list
                print_time = time.time()
                if print_time > pre_time+0.3:
                    logger_info.info("altitude of LIDAR:{}".format(distance))
                    pre_time = print_time
                altitude_list.append(distance)
                
            elif priority == "POSITION":
                try:
                    position = await asyncio.wait_for(self.get_position_alt(), timeout = 0.8)
                except asyncio.TimeoutError:
                    logger_info.info("GPS can't catch or pixhawk might have some error")
                    altitude_list =[]
                    return altitude_list
                print_time = time.time()
                if print_time > pre_time+0.3:
                    logger_info.info("altitude of POSITION:{}".format(position))
                    pre_time = print_time
                altitude_list.append(position)
        return altitude_list
            

    def IQR_removal(self, data):
        data.sort()
        l = len(data)
        flag = l%4
        value_25 = l//4
        if flag == 0:
            quartile_25 = (data[value_25]+data[value_25+1])/2
            quartile_75 = (data[value_25*3]+data[value_25*3+1])/2
        elif flag == 1:
            quartile_25 = (data[value_25]+data[value_25+1])/2
            quartile_75 = (data[value_25*3+1]+data[value_25*3+2])/2
        elif flag == 2:
            quartile_25 = data[value_25]
            quartile_75 = data[value_25*3]
        elif flag == 3:
            quartile_25 = data[value_25]
            quartile_75 = data[value_25*3+1]
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


    def fuse(self):
        
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.fuse_pin, GPIO.OUT, initial=GPIO.HIGH)
            logger_info.info("Fuse start")

            GPIO.output(self.fuse_pin, 0)
            logger_info.info("-- Fusing")

            time.sleep(self.fuse_time)
            logger_info.info("Fused")

            GPIO.output(self.fuse_pin, 1)
        
        except:
            GPIO.output(self.fuse_pin, 1)
            

    def fuse(self):
        
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.fuse_pin, GPIO.OUT, initial=GPIO.LOW)
            logger_info.info("Light start")

            GPIO.output(self.fuse_pin, 1)
            logger_info.info("-- Lighting")

            time.sleep(self.fuse_time)
            logger_info.info("Lighted")

            GPIO.output(self.fuse_pin, 0)
        
        except:
            GPIO.output(self.fuse_pin, 0)
            
            
    async def health_check(self):
        
        logger_info.info("Waiting for drone to have a global position estimate...")
        
        health_true_count = 0

        async for health in self.pix.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                logger_info.info("gps ok")
                health_true_count += 1
            else:
                logger_info.info("gps ng")
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
            self.cycle_mission_progress(),
            self.cycle_position_lat_lng(),
            self.cycle_lidar(),
            self.cycle_wait_mission_finished()
        ]
        tasks = asyncio.gather(*main_coroutines)
        cancel_task = asyncio.create_task(self.tasks_cancel(tasks))
        await asyncio.gather(tasks, cancel_task, return_exceptions=True)


    async def gather_land_coroutines(self):

        land_coroutines = [
            self.cycle_pitch_roll(),
            self.cycle_is_in_air(),
            self.cycle_land()
        ]
        tasks = asyncio.gather(*land_coroutines)
        cancel_task = asyncio.create_task(self.tasks_cancel(tasks))
        await asyncio.gather(tasks, cancel_task, return_exceptions=True)


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
        self.is_tasks_cancel_ok = True 


    async def cycle_land(self):

        await self.pix.action.land
        while True:
            if abs(float(self.roll_deg)) > 60 or abs(float(self.pitch_deg)) > 60:
                logger_info.info("Hit the target!")
                await self.kill_forever()
            elif self.is_in_air == False:
                logger_info.info("Landed!")
                break
            await asyncio.sleep(0.01)
        self.is_tasks_cancel_ok = True 
        

    async def tasks_cancel(self, tasks):
        
        while True:
            await asyncio.sleep(1)
            if self.is_tasks_cancel_ok:
                break
        tasks.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


    async def goto_location(self, lat, lng, abs_alt):

        logger_info.info("Setting goto_location...")
        await self.pix.action.goto_location(lat, lng, abs_alt, 0)
        logger_info.info("Going to location...")
        await asyncio.sleep(20)


    async def estimate_target_position(self):

        async for d in self.pix.telemetry.distance_sensor(): #? 測れなかったらどうしよう
            lidar_height = d.current_distance_m
            logger_info.info(f"current height:{lidar_height}m")
            break

        async for heading in self.pix.telemetry.heading():
            heading_deg = heading.heading_deg
            logger_info.info(f"current heading: {heading_deg}") 
            break
        
        self.camera.take_pic()
        self.image_res = self.camera.detect_center()
        logger_info.info('percent={}, center={}'.format(self.image_res['percent'], self.image_res['center']))
        if self.image_res['percent'] <= 0.001:
            logger_info.info(f"Failed image navigation")
            await self.land()
        else:
            logger_info.info(f"Target detected!")
            x_m, y_m = self.camera.get_target_position(self, lidar_height)

            # self.east_m = 1/np.sqrt(2)*(y_m-x_m)*np.cos(heading_deg*np.pi/180)-1/np.sqrt(2)*(y_m+x_m)*np.sin(heading_deg*np.pi/180)
            # self.north_m = 1/np.sqrt(2)*(y_m-x_m)*np.sin(heading_deg*np.pi/180)+1/np.sqrt(2)*(y_m+x_m)*np.cos(heading_deg*np.pi/180)
            self.east_m = -np.cos(heading_deg*np.pi/180)*x_m-np.sin(heading_deg*np.pi/180)*y_m
            self.north_m = -np.sin(heading_deg*np.pi/180)*x_m+np.cos(heading_deg*np.pi/180)*y_m

            logger_info.info(f"go to the red position:北に{self.north_m}m,東に{self.east_m}")
        

    async def start_offboard_ned(self):

        logger_info.info("-- Setting initial setpoint")
        await self.pix.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0 , 0.0))
        await self.pix.offboard.start()

    
    async def image_navigation_offboard_ned(self):

        await self.pix.offboard.set_position_ned(
            PositionNedYaw(self.north_m, self.east_m, 0.0, 0.0))
        

    async def calc_red_position(self):
        lat_deg_per_m = 0.000008983148616
        lng_deg_per_m = 0.000008983668124

        await self.estimate_target_position()

        async for position in self.pix.telemetry.position():
            lat_now = position.latitude_deg
            lng_now = position.longitude_deg
            abs_alt = position.absolute_altitude_m
            break
        red_lat = lat_now+self.north_m*lat_deg_per_m
        red_lng = lng_now+self.east_m*lng_deg_per_m
        is_red_right_below = False
        if abs(self.image_res['center'][0])<0.2 and abs(self.image_res['center'][1])<0.2:
            is_red_right_below = True
        return red_lat, red_lng, abs_alt, is_red_right_below
        

    async def image_navigation_goto(self):

        red_lat, red_lng, abs_alt= await self.calc_red_position()
        logger_info.info(f"[go to] red_lat:{red_lat}, red_lng:{red_lng}, abs_alt:{abs_alt}")
        await self.pix.action.goto_location(red_lat, red_lng, abs_alt, 0)
        await asyncio.sleep(10)
        

    async def stop_offboard(self):
        logger_info.info("-- Stopping offboard")
        try:
            await self.pix.offboard.stop()
        except OffboardError as error:
            logger_info.info(f"Stopping offboard mode failed \
                    with error code: {error._result.result}")
            await self.land()


    async def kill_forever(self):

        while True:
            await self.pix.action.kill()
            await asyncio.sleep(0.1)