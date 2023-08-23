import RPi.GPIO as GPIO
import serial
import asyncio
from logger_lib import logger_info, logger_debug
import time
from mavsdk import System

class Lora:
    
    def __init__(self):
        
        self.power_Pin = 4
        self.pix = System()
        self.CRLF = "\r\n"
        self.is_power_on = False
        self.connect_counter = 0
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.rst, GPIO.OUT)
        GPIO.setup(self.power, GPIO.OUT)
        
        
    async def serial_connect(self):
    
        connect_counter = 0
        while True:
            try:
                self.serial = serial.Serial('/dev/ttyS0', 115200, timeout=0.6)
            except:
                logger_info.info ("Serial port error. Waiting.")
                connect_counter += 1
                if connect_counter >= 10:
                    break
                await asyncio.sleep(3)
            else:
                break
        logger_info.info("Serial port OK.")
        self.operation_write()
        await self.write(("Lora start").encode())
        logger_info.info("Lora READY")
    
        
    def operation_write(self):
        
        self.serial.write(b'2\r\n')
        time.sleep(1)
        self.serial.write(b'z\r\n')
        time.sleep(1)
        
       
    async def power_off(self):
        GPIO.output(self.power_Pin, GPIO.LOW)
        self.is_power_on = False
        print("Lora power off")
        await asyncio.sleep(1)
        

    async def power_on(self):
        GPIO.output(self.power_Pin, GPIO.HIGH)
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
        
        
    async def send_gps(self):
        
        lat_deg, lng_deg, alt_deg = await self.get_gps()
        if self.is_lora_power_on:
            lat = "lat:" + str(lat_deg)
            lng = "lng:" + str(lng_deg)
            alt = "alt:" + str(alt_deg)
            await self.write(lat.encode())
            logger_info.info(lat)
            await self.write(lng.encode())
            logger_info.info(lng)
            await self.write(alt.encode())
            logger_info.info(alt)
            
            
    async def get_gps(self):
    
        lat, lng, alt = "0", "0", "0"
        while True:
            try:
                await asyncio.wait_for(self.gps(self.pix), timeout=0.8)
            except asyncio.TimeoutError:
                logger_info.info("Can't catch GPS")
                lat = "error"
                lng = "error"
                alt = "error"
            return lat, lng, alt
            
            
    async def gps(self):
        
        global lat, lng, alt
        async for position in self.pix.telemetry.position():
                logger_info.info(position)
                lat = str(position.latitude)
                lng = str(position.longitude)
                alt = str(position.absolute_altitude_m)
                break