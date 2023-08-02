import serial
import time
from mavsdk import System
import asyncio

system_address = "serial:///dev/ttyACM0:115200"

async def main():
    drone = System()
    print("Waiting for drone to connect...")
    await drone.connect(system_address)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
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
    print("READY")
    async for distance in drone.telemetry.distance_sensor():
        buf = str(distance.current_distance_m)+"\r\n"
        print(buf)
        break
    print("writing")
    ser.write(buf.encode())
    print("write")

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main())