import asyncio
import serial
from mavsdk import System


hovering_hight = 5
ser = serial.Serial("/dev/ttyAMA0", 115200) #dev/ttyACM0:115200 ?


def getTFminiData():
    while True:
        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)
            ser.reset_input_buffer()
            if recv[0] == 'Y' and recv[1] == 'Y': # 0x59 is 'Y'
                low = int(recv[2].encode('hex'), 16)
                high = int(recv[3].encode('hex'), 16)
                distance = low + high * 256
                return distance
            
            
def get_hight():
    try:
        if ser.is_open == False:
            ser.open()
        getTFminiData()
    except KeyboardInterrupt:   # Ctrl+C
        if ser != None:
            ser.close()
            

def get_xy_position(drone):
    position = drone.telemetry.position()
    return position
                
                
async def run():

    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

    status_text_task = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming....")
    await drone.action.arm()
    print("-- Armed")

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(5)
    
    print("-- Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    position = get_xy_position()
    latitude_deg, longitude_deg = position[0], position[1]
    hovering_hight += absolute_altitude
    await drone.action.goto_location(latitude_deg, longitude_deg, hovering_hight, 0)
    print("-- Reached the hovering hight")
    
    await asyncio.sleep(5)

    print("-- Landing....")
    await drone.action.land()
    
async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
