import RPi.GPIO as GPIO
import serial
import asyncio
from logger_lib import logger_info
import time
from mavsdk import System


class Lora:
    
    def __init__(self,
                 lora_power_pin):
        
        self.lora_power_pin = lora_power_pin
        self.pix = System()
        self.CRLF = "\r\n"
        self.is_lora_power_on = False
        self.connect_counter = 0
        self.power = 0
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.power, GPIO.OUT)
        
        logger_info.info("Lora initialized")
        
        
    def serial_connect(self):
    
        connect_counter = 0
        while True:
            try:
                self.serial = serial.Serial('/dev/ttyS0', 115200, timeout=0.6)
            except:
                logger_info.info ("Serial port error. Waiting.")
                connect_counter += 1
                if connect_counter >= 10:
                    break
                time.sleep(3)
            else:
                break
        logger_info.info("Serial port OK.")
        self.operation_write()
        self.serial.write(b"Lora start\r\n")
        logger_info.info("Lora READY")
    
        
    def operation_write(self):
        
        self.serial.write(b'2\r\n')
        time.sleep(1)
        self.serial.write(b'z\r\n')
        time.sleep(1)
        
       
    async def power_off(self):
        
        GPIO.output(self.lora_power_pin, GPIO.LOW)
        self.is_lora_power_on = False
        logger_info.info("Lora power off")
        await asyncio.sleep(1)
        

    async def power_on(self):
        
        GPIO.output(self.lora_power_pin, GPIO.HIGH)
        logger_info.info("Lora power on")
        self.serial_connect()

        self.is_lora_power_on = True
            
    
    async def write(self, message: str):
        
        msg_send = str(message) + self.CRLF
        self.serial.write(msg_send.encode("ascii"))
        await asyncio.sleep(4)
        
        
    async def send_gps(self):
        
        lat_deg, lng_deg, alt_deg = await self.get_gps()
        if self.is_lora_power_on:
            lat = "lat:" + str(lat_deg)
            lng = "lng:" + str(lng_deg)
            alt = "alt:" + str(alt_deg)
            await self.write(lat.encode())
            logger_info.info(lat)
            await asyncio.sleep(0)
            await self.write(lng.encode())
            logger_info.info(lng)
            await asyncio.sleep(0)
            await self.write(alt.encode())
            logger_info.info(alt)
            await asyncio.sleep(0)
            
            
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
                lat = str(position.latitude_deg)
                lng = str(position.longitude_deg)
                alt = str(position.relative_altitude_m)
                break