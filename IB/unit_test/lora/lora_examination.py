import serial
import time
from mavsdk import System
import asyncio

system_address = "serial:///dev/ttyACM0:115200"
lat = ""
lng = ""
alt = ""

def Serial_connect():
    while True:
        try:
            ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
        except:
            print ("Serial port error. Waiting.")
            time.sleep(5)
        else:
            break
    print ("Serial port OK.")
    ser.write(b'1\r\n')
    time.sleep(1)
    ser.write(b'z\r\n')
    time.sleep(1)
    ser.write(("Let's GO\r\n").encode())
    time.sleep(1)
    print("READY")
    return ser

async def Write_GPS(ser):
    while True:
        print("writing")
        latitude = "lat:" + lat
        longitude = "lng:" + lng
        altitude = "alt:" + alt
        ser.write(latitude)
        ser.write(longitude)
        ser.write(altitude)
        await asyncio.sleep(6)
        
async def Get_GPS(drone):
    global lat, lng, alt
    while True:
        try:
            await asyncio.wait_for(GPS(drone), timeout=0.8)
        except asyncio.TimeoutError:
            print("Can't catch GPS")
            lat = "error"
            lng = "error"
            alt = "error"
        await asyncio.sleep(1)
        
async def GPS(drone):
    global lat, lng, alt
    async for position in drone.telemetry.position():
            print(position)
            lat = str(position.latitude)
            lng = str(position.longitude)
            alt = str(position.absolute_altitude_m)
            break
        
async def main():
    drone = System()
    print("Waiting for drone to connect...")
    await drone.connect(system_address)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break
    serial = Serial_connect()
    get_GPS_task = asyncio.ensure_future(Get_GPS(drone))
    write_task = asyncio.ensure_future(Write_GPS(serial))
    await get_GPS_task
    await write_task
    
if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main())