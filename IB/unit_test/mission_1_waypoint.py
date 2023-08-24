import asyncio
from Library.pixhawk import Pixhawk

# parameters---------------------
waypoint = [40.19373, 140.05923]
altitude = 5
speed = 5
#--------------------------------

async def run():

    pixhawk = Pixhawk()

    main_coroutines = [
        pixhawk.cycle_flight_mode(),
        pixhawk.cycle_position_lat_lng(), 
        pixhawk.cycle_lidar(),
        pixhawk.cycle_show(),
        pixhawk.mission_land()
        ]
    
    await pixhawk.connect()

    await pixhawk.upload_mission(waypoint, altitude, speed)

    # await pixhawk.health_check()

    await pixhawk.arm()

    await pixhawk.start_mission()

    await asyncio.gather(*main_coroutines)

if __name__ == "__main__":

    asyncio.run(run())