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
        self.power = 7
        self.CRLF = "\r\n"
        self.is_power_on = False
        self.connect_counter = 0
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.rst, GPIO.OUT)
        GPIO.setup(self.power, GPIO.OUT)
        
    def Serial_Connect(self):
        while True:
            try:
                self.serial = serial.Serial("/dev/ttyS0", 115200, timeout=1)
            except:
                print ("Serial port error. Waiting.")
                self.connect_counter += 1
                if self.connect_counter == 100:
                    break
                time.sleep(3)
            else:
                break
        print ("Serial port OK.")
        self.write(b'2'+self.CRLF)
        time.sleep(1)
        self.write(b'z'+self.CRLF)
        time.sleep(1)
        self.write(("Lora start"+self.CRLF).encode())
        print("Lora READY")
       
    async def power_off(self):
        GPIO.output(self.power, GPIO.LOW)
        self.is_power_on = False
        print("Lora power off")
        await asyncio.sleep(1)

    async def power_on(self):
        GPIO.output(self.power, GPIO.HIGH)
        print("Lora power on")
        await self.write("processor")
        await self.write("start")

        self.is_power_on = True
        
    async def start_communication(self):
    
    async def write(self, message: str):
        msg_send = str(message) + self.CRLF
        self.serial.write(msg_send.encode("ascii"))
        await asyncio.sleep(4)