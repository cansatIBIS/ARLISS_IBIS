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
            print(f"-- Connected to drone!")
            break

    # print("Waiting for drone to have a global position estimate...")
    # async for health in drone.telemetry.health():
    #     if health.is_global_position_ok and health.is_home_position_ok:
    #         print("-- Global position estimate OK")
    #         break

    print("-- Arming")
    await drone.action.arm()
    
    # drone.is_armed = False
    # drone.is_armed = drone.telemetry.Telemetry.armed()
    
    print("is_arm : {}".format(drone.telemetry.is_armed))

    # print("-- Taking off")
    # await drone.action.takeoff()

    # await asyncio.sleep(10)

    # print("-- Landing")
    # await drone.action.land()

    # status_text_task.cancel()

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())