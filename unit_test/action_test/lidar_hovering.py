import asyncio
import pigpio
import time
from mavsdk import System

#Lidar関係
RX = 23
pi = pigpio.pi()
pi.set_mode(RX, pigpio.INPUT)
pi.bb_serial_read_open(RX, 115200) 


#高さ指定
hovering_hight = 5


ser = serial.Serial("/dev/ttyAMA0", 115200) #dev/ttyACM0:115200 ?


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
    
    status_text_task.cancel()
    
async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())