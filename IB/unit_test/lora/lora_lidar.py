import serial
import time
from mavsdk import System
import asyncio

system_address = "serial:///dev/ttyACM0:115200"
dist = 0

def Serial_connect():
    while True:
        try:
            ser = serial.Serial('/dev/ttyS0' ,19200,timeout=1)
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
    print("READY")
    return ser

async def Write_distance(ser):
    while True:
        bufw = dist
        ser.write((bufw+"\r\n").encode())
        await asyncio.sleep(3)
        
async def Connect_pix ():
    drone = System()
    print("Waiting for drone to connect...")
    await drone.connect(system_address)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    return drone
        
async def Get_distance(drone):
    global dist
    async for distance in drone.telemetry.distance_sensor():
        dist = distance
        await asyncio.sleep(3)
        
async def main():
    drone = Connect_pix()
    serial = Serial_connect()
    get_dist_task = asyncio.create_task(Get_distance(drone))
    write_task = asyncio.create_task(Write_distance(serial))
    await get_dist_task
    await write_task
    
if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main())