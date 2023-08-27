import os
import sys
import picamera
import cv2
import numpy as np
import datetime
from logger import logger_info


class Camera:

    def __init__(self,
                 hsv_min_1,
                 hsv_max_1,
                 hsv_min_2,
                 hsv_max_2,
                 pixel_number_x = 3296,
                 pixel_number_y = 2521,
                 pixel_size = 1.12,
                 focal_length = 3.04,
                 image_path = "/home/pi/ARLISS_IBIS/IB/Images/" 
                 + str(os.path.splitext(os.path.basename(sys.argv[0]))[0])
                 + "_"
                 + datetime.datetime.now()
                 + ".jpg"):
        
        self.camera = picamera.PiCamera()

        self.hsv_min_1 = hsv_min_1
        self.hsv_max_1 = hsv_max_1
        self.hsv_min_2 = hsv_min_2
        self.hsv_max_2 = hsv_max_2
        self.pixel_number_x = pixel_number_x
        self.pixel_number_y = pixel_number_y
        self.pixel_size = pixel_size
        self.focal_length = focal_length
        self.image_path = image_path

        self.img  = None
        self.center_px = None

        logger_info.info("Camera initialized")


    def take_pic(self):
        logger_info.info("taking pic...: {}".format(self.image_path))
        self.camera.capture(self.image_path)

    def save_detected_img(self):
        cv2.circle(self.img, (int(self.center_px[0]), int(self.center_px[1])), 30, (0, 200, 0),
                thickness=3, lineType=cv2.LINE_AA)
        cv2.imwrite(self.image_path, self.img)

    def detect_center(self):
        self.img = cv2.imread(self.image_path) # 画像を読み込む
        
        height, width = self.img.shape[:2] # 画像のサイズを取得する
        print(f"height:{height}, width:{width}")

        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV) # 色基準で2値化する

        # 色の範囲を指定する
        mask1 = cv2.inRange(hsv, self.hsv_min_1, self.hsv_max_1)

        # 赤色のHSVの値域2
        mask2 = cv2.inRange(hsv, self.hsv_min_2, self.hsv_max_2)

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
            self.save_detected_img(self.file_path, self.img, ((1-res['center'][0])*width/2, (1-res['center'][1])*height/2))
        
        return res
    
    def get_image_info(self):
