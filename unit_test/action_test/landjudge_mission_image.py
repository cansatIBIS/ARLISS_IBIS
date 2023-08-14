#!/usr/bin/env python3
# 北に20m→南に40m
import time
import RPi.GPIO as GPIO
import asyncio
import picamera
import cv2
import numpy as np
import datetime
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.offboard import (OffboardError, PositionNedYaw)
from logger import logger_info, logger_debug

# パラメータ--------------------------------
goal = [35.7927147,139.8908122]
recognition_height = 5 # 画像認識開始時の高度
#-----------------------------------------

#　picameraの仕様--------------------------
pixel_number_x = 3296 #[mm]
pixel_number_y = 2521
pixel_size = 1.12 #[um]
f = 3.04 #[mm]
# ----------------------------------------

is_landed = False
PIN = 5
is_fused = False
is_mission_finished = False
mode = None

async def run():
    
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

    logger_info.info("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print_alt_task = asyncio.create_task(print_alt(drone))
    land_judge_task = asyncio.create_task(land_judge(drone))
    
    await print_alt_task
    await land_judge_task

    # print_mission_progress_task = asyncio.ensure_future(
    #     print_mission_progress(drone))

    # running_tasks = [print_mission_progress_task]
    # termination_task = asyncio.ensure_future(
    #     observe_is_in_air(drone, running_tasks))
    get_log_task = asyncio.ensure_future(get_log(drone))
    img_navigation_task = asyncio.ensure_future(img_navigation(drone))

    mission_items = []
    mission_items.append(MissionItem(goal[0],
                                    goal[1],
                                    recognition_height, # rel_alt
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

    # await termination_task
    await get_log_task
    await img_navigation_task


# async def print_mission_progress(drone):
#     async for mission_progress in drone.mission.mission_progress():
#         logger_info.info(f"Mission progress: "
#               f"{mission_progress.current}/"
#               f"{mission_progress.total}")


# async def observe_is_in_air(drone, running_tasks):
#     was_in_air = False
#     async for is_in_air in drone.telemetry.in_air():
#         if is_in_air:
#             was_in_air = is_in_air

#         if was_in_air and not is_in_air:
#             for task in running_tasks:
#                 task.cancel()
#                 try:
#                     await task
#                 except asyncio.CancelledError:
#                     pass
#             await asyncio.get_event_loop().shutdown_asyncgens()

#             return
        
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

async def land_judge(drone):
    global is_landed
    start_time = time.time()
    while True:
        time_now = time.time()
        await asyncio.sleep(0)
        if time_now-start_time < 30:
            try :
                alt_now = await(asyncio.wait_for(get_distance_alt(drone), timeout = 0.8))
            except asyncio.TimeoutError:
                continue
                
            if is_judge_alt(alt_now):
                true_dist = IQR_removal(await get_alt_list(drone, "LIDAR"))
                if len(true_dist) == 0:
                    continue
                try:
                    ave = sum(true_dist)/len(true_dist)
                except ZeroDivisionError as e:
                    logger_info.info(e)
                    continue
                
                if is_low_alt(ave):
                    for distance in true_dist:
                        if abs(ave-distance) > 0.01:
                            logger_info.info("-- Moving")
                            break
                    else:
                        true_posi = IQR_removal(await get_alt_list(drone, "POSITION"))
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
            if is_landed:
                logger_info.info("-- Timer Judge")
                break
                
    
    logger_info.info("-- Land judge finish")

        
def is_low_alt(alt):
    if alt < 1:
        return True
    else:
        return False


def is_judge_alt(alt):
    if alt < 15:
        logger_info.info("-- Land judge start")
        return True
    else:
        return False
        
        
async def get_alt_list(drone, priority):
    altitude_list = []
    iter = 0
    while True:
        if priority == "LIDAR":
            try :
                distance = await asyncio.wait_for(get_distance_alt(drone), timeout = 0.8)
            except asyncio.TimeoutError:
                logger_info.info("Distance sensor might have some error")
                altitude_list =[]
                return altitude_list
            altitude_list.append(distance)
            
        elif priority == "POSITION":
            try:
                position = await asyncio.wait_for(get_position_alt(drone), timeout = 0.8)
            except asyncio.TimeoutError:
                logger_info.info("Pixhawk might have some error")
                altitude_list =[]
                return altitude_list
            altitude_list.append(position)
            
        iter += 1
        if iter >= 30:
            break
    return altitude_list
        

def IQR_removal(data):
    try:
        data.sort()
        quartile_25 = (data[7]+data[8])/2
        quartile_75 = (data[22]+data[23])/2
        IQR = quartile_75-quartile_25
        true_data = [i for i in data if quartile_25-1.5*IQR <= i <= quartile_75+1.5*IQR]
    except IndexError as e:
        logger_info.info(e)
        true_data = []
    return true_data


async def print_alt(drone):
    while True:
        try:
            position = await asyncio.wait_for(get_position_alt(drone), timeout = 0.8)
            logger_info.info("altitude:{}".format(position))
        except asyncio.TimeoutError:
            logger_info.info("Pixhawk might have some error")
        if is_landed:
            return
        await asyncio.sleep(0)
        

async def get_distance_alt(drone):
    async for distance in drone.telemetry.distance_sensor():
        return distance.current_distance_m
    

async def get_position_alt(drone):
    async for position in drone.telemetry.position():
        return position.absolute_altitude


def wait():
    logger_info.info("-- Waiting")
    time.sleep(5)
    logger_info.info("5s passed")
    time.sleep(5)
    logger_info.info("10s passed")
    time.sleep(5)
    logger_info.info("15s passed")


def fusing():
    global is_fused
    try:
        logger_info.info("-- Fuse start")
        time.sleep(3)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(PIN, GPIO.OUT)

        GPIO.output(PIN, 0)
        logger_info.info("-- Fusing")

        time.sleep(5.0)
        logger_info.info("--Nichrome Wire Fused")

        GPIO.output(PIN, 1)
        
        print("Waiting 5s...")
        time.sleep(5.0)
        is_fused = True
    
    except:
        GPIO.output(PIN, 1)

async def img_navigation(drone):
    while True:
        await asyncio.sleep(1)
        mission_finished = await drone.mission.is_mission_finished()
        if mission_finished:
            logger_info.info("mission finished")
            break

    asyncio.sleep(5)

    camera = picamera.PiCamera()
    logger_info.info('キャメラ初期化完了')

    async for d in drone.telemetry.distance_sensor(): #? 測れなかったらどうしよう
        lidar_height = d.current_distance_m
        logger_info.info(f"理想:{recognition_height}m,実際:{lidar_height}m")
        break
    async for heading in drone.telemetry.heading():
        logger_info.info(f"方位: {heading} 45であって欲しい") 
        break
    logger_info.info("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, recognition_height-lidar_height, 0.0)) #　方位は北東を向いているはず
    await drone.offboard.start()
    logger_info.info(f"高度{recognition_height}mの地点に向かいます")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(5)

    file_No = 0
    non_rec_count = 0
    while True:
        file_path = '/home/pi/ARLISS_IBIS/Images/image_test{}_{}.jpg'.format(datetime.datetime.now(),file_No)
        file_No += 1

        logger_info.info("taking pic...: {}".format(file_path))
        logger_info.info(camera,file_path) # 写真を撮る
        res = detect_center(file_path) # 赤の最大領域の占有率と重心を求める

        logger_info.info('percent={}, center={}'.format(res['percent'], res['center']))

        asyncio.sleep(1)

        distance = recognition_height
        a = pixel_number_x*pixel_size/1000 # 画像(ピクセル単位)の横の長さ[mm]
        b = pixel_number_y*pixel_size/1000 # 画像(ピクセル単位)の縦の長さ[mm]
        image_x = distance*a/f # 画像の横の距離[m]
        image_y = distance*b/f # 画像の縦の距離[m]
        x_m = res['center'][0]*image_x/2
        y_m = res['center'][1]*image_y/2

        if res['center'][0] or res['center'][1] is None:
            if recognition_height-non_rec_count<3: #　地面に近すぎたらland
                logger_info.info("-- Stopping offboard")
                try:
                    await drone.offboard.stop()
                except OffboardError as error:
                    logger_info.info(f"Stopping offboard mode failed \
                            with error code: {error._result.result}")
                logger_info.info("画像認識失敗、着陸します") 
                await drone.action.land()
            else:
                non_rec_count += 1
                logger_info.info(f"高度を{recognition_height-non_rec_count}mにします")
                await drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, non_rec_count, 0.0))
                await asyncio.sleep(5)
        else:
            logger_info.info("画像認識完了")
            break
    logger_info.info(f"go to the red position:北に{y_m}m,東に{-x_m}")

    await drone.offboard.set_position_ned(
            PositionNedYaw(y_m, -x_m, non_rec_count, 0.0)) #? 方位が違うかも
    await asyncio.sleep(10)

    logger_info.info("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        logger_info.info(f"Stopping offboard mode failed \
                with error code: {error._result.result}")
    logger_info.info("画像認識成功、着陸します") 
    await drone.action.land()

def take_pic(camera,file_path):
    camera.capture(file_path)

def save_detected_img(file_path, img, center_px):
    cv2.circle(img, (int(center_px[0]), int(center_px[1])), 30, (0, 200, 0),
            thickness=3, lineType=cv2.LINE_AA)
    cv2.imwrite(file_path, img)

def detect_center(file_path):
    img = cv2.imread(file_path) # 画像を読み込む
    
    height, width = img.shape[:2] # 画像のサイズを取得する

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # 色基準で2値化する

    # 色の範囲を指定する
    hsv_min = np.array([0,145,0])
    hsv_max = np.array([5,255,255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    # 赤色のHSVの値域2
    hsv_min = np.array([150,110,0]) #カメラ故障のため，0→150へ変更
    hsv_max = np.array([179,255,255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

    mask = mask1 + mask2

    # 非ゼロのピクセルが連続してできた領域を検出する
    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

    #　画像の背景の番号は 0 とラベリングされているので、実際のオブジェクトの数は nlabels - 1 となる
    nlabels = nlabels - 1
    labels = np.delete(labels, obj=0, axis=0)
    stats = np.delete(stats, obj=0, axis=0)
    centroids = np.delete(centroids, obj=0, axis=0)
    centroids[:,0] = (width/2 - centroids[:,0]) / width*2
    centroids[:,1] = (height/2 - centroids[:,1]) / height*2
    percent = stats[:,4] / (height*width)
    
    res = {}

    if nlabels == 0:
        res['height'] = None
        res['width'] = None
        res['percent'] = 0
        res['center'] = None
    else:
        max_index = np.argmax(percent)
        res['height'] = height
        res['width'] = width
        res['percent'] = percent[max_index]
        res['center'] = centroids[max_index]
        save_detected_img(file_path, img, ((1-res['center'][0])*width/2, (1-res['center'][1])*height/2))
    
    return res


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())