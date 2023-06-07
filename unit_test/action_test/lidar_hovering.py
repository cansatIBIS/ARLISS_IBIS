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


def getTFminiData():
	while True:
		#print("#############")
		time.sleep(0.05)	#change the value if needed
		(count, recv) = pi.bb_serial_read(RX)
		if count > 8:
			for i in range(0, count-9):
				if recv[i] == 89 and recv[i+1] == 89: # 0x59 is 89
					checksum = 0
					for j in range(0, 8):
						checksum = checksum + recv[i+j]
					checksum = checksum % 256
					if checksum == recv[i+8]:
						distance = recv[i+2] + recv[i+3] * 256
						strength = recv[i+4] + recv[i+5] * 256
						if distance <= 1200 and strength < 2000:
							print(distance, strength) 
            
            
def get_hight():
    try:
        getTFminiData()
    except:  
        pi.bb_serial_read_close(RX)
        pi.stop()
            

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