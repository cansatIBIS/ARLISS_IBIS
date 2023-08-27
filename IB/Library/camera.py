import asyncio
import picamera
import cv2
import numpy as np
import datetime
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.offboard import (OffboardError, PositionNedYaw)
from logger import logger_info


class Camera:

    def __init__(self,
                 pixel_number_x = 3296,
                 pixel_number_y = 2521,
                 pixel_size = 1.12,
                 focal_length = 3.04
                 image_path = "/home/pi/ARLISS_IBIS/IB/Images):
        
        self.camera = picamera.PiCamera()
        
        self.pixel_number_x = pixel_number_x
        self.pixel_number_y = pixel_number_y
        self.pixel_size = pixel_size
        self.focal_length = focal_length

        logger_info.info("Camera initialized")


    def take_pic(self):
        self.camera.capture(file_path)