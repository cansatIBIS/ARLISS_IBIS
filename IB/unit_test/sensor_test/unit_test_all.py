import spidev               
import time                         
import sys    
import RPi.GPIO as GPIO
import picamera
from mavsdk import System
import serial
import cv2
import numpy as np
import datetime


fuse_Pin = 3
lora_power_pin = 4