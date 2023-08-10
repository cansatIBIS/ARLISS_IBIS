import asyncio
from mavsdk import System


async def run():

    drone = System()
    print("HEY")
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    # await drone.connect(system_address="udp://:14540")

    

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break
        
    # gps_info_0 = drone.telemetry.gps_info()
    # while True:
    #     gps_info = drone.telemetry.gps_info()
    #     if gps_info != gps_info_0:
    #         break
    

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        print(health.is_global_position_ok
              )
        print(health.is_home_position_ok)
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- arming")
    await drone.action.arm()
    
    # drone.is_armed = False
    # drone.is_armed = drone.telemetry.Telemetry.armed()
    
    print("-- Armed")

    # print("-- Taking off")
    # await drone.action.takeoff()

    # await asyncio.sleep(10)

    # print("-- Landing")
    # await drone.action.land()

    # status_text_task.cancel()

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
