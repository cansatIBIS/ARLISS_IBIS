# import os
# import sys
import RPi.GPIO as GPIO
import serial
# sys.path.append(os.getcwd())
import asyncio

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
        
    async def Serial_Connect(self):
        while True:
            try:
                self.serial = serial.Serial("/dev/ttyS0", 115200, timeout=1)
            except:
                print ("Serial port error. Waiting.")
                self.connect_counter += 1
                if self.connect_counter == 100:
                    break
                await asyncio.sleep(3)
            else:
                break
        print ("Serial port OK.")
        self.write(b'2'+self.CRLF)
        await asyncio.sleep(1)
        self.write(b'z'+self.CRLF)
        await asyncio.sleep(1)
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
        
    async def start_communication(self, light):
        while True:
            if light.is_released:
                await self.power_on()
                return
            await self.power_off()
    
    async def write(self, message: str):
        msg_send = str(message) + self.CRLF
        self.write(msg_send.encode("ascii"))
        await asyncio.sleep(4)
        
    async def send_GPS(self, pix):
        while True:
            if self.is_power_on:
                lat = "lat:" + str(pix.latitude_deg)
                lng = "lng:" + str(pix.longitude_deg)
                alt = "alt:" + str(pix.absolute_altitude_m)
                await self.write(lat)
                await self.write(lng)
                await self.write(alt)
            await asyncio.sleep(10)