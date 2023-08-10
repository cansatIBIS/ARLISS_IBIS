import asyncio
import picamera
import cv2
import numpy as np
import time
from mavsdk import System
import datetime

#　picameraの仕様-------------------
pixel_number_x = 3296 #[mm]
pixel_number_y = 2521
pixel_size = 1.12 #[um]
f = 3.04 #[mm]
# ---------------------------------



    
async def run():
    print("waiting...")
    time.sleep(10)
    camera = picamera.PiCamera()
    drone = System()
    print('キャメラ初期化完了')

    file_path = '/home/pi/ARLISS_IBIS/Images/image_test{}.jpg'.format(datetime.datetime.now())

    print("taking pic...: {}".format(file_path))
    take_pic(camera,file_path) # 写真を撮る
    res = detect_center(file_path) # 赤の最大領域の占有率と重心を求める

    # ログの出力
    print('percent={}, center={}'.format(res['percent'], res['center']))

    distance = 2.65 # [m]
    # async for d in drone.telemetry.distance_sensor():
    #     distance = d.current_distance_m
    #     break
    time.sleep(1)



    a = pixel_number_x*pixel_size/1000 # 画像(ピクセル単位)の横の長さ[mm]
    b = pixel_number_y*pixel_size/1000 # 画像(ピクセル単位)の縦の長さ[mm]
    image_x = distance*a/f # 画像の横の距離[m]
    image_y = distance*b/f # 画像の縦の距離[m]
    res = detect_center(file_path)
    x_m = res['center'][0]*image_x/2
    y_m = res['center'][1]*image_y/2
    print(f"x={x_m},y={y_m}")



    


    cv2.destroyAllWindows()

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
    hsv_min = np.array([0,110,0]) #カメラ故障のため，0→150へ変更
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
    asyncio.run(run())
    