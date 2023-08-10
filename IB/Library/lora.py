import os
import sys
import mavsdk
import RPi.GPIO as GPIO
import serial
sys.path.append(os.getcwd())
import asyncio
import struct
import time

class Lora:
    def __init__(self):
        self.rst = 17
        self.power = 4
        self.CRLF = "\r\n"
        self.serial = serial.Serial("/dev/ttyS0", 115200, timeout=1)
        self.is_on = False
        self.is_out = False
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.rst, GPIO.OUT)
        GPIO.setup(self.power, GPIO.OUT)
        
    async def power_off(self):
        """cut power for lora"""
        GPIO.output(self.power, GPIO.LOW)
        self.is_on = False
        await asyncio.sleep(1)

    async def power_on(self):
        """start lora"""
        GPIO.output(self.power, GPIO.HIGH)

        GPIO.output(self.rst, GPIO.LOW)
        await asyncio.sleep(2)
        GPIO.output(self.rst, GPIO.HIGH)
        await asyncio.sleep(2)
        print("lora power on")
        await self.write("processor")
        await self.write("start")

        self.is_on = True
        
    def Serial_Connect(self):
        while True:
            try:
                ser = serial.Serial('/dev/ttyS0', 115200, timeout=3)
            except:
                print ("Serial port error. Waiting.")
                time.sleep(5)
            else:
                break
        print ("Serial port OK.")