import asyncio

from Library.Pixhawk import Pixhawk
from Library.logger_lib import logger_info

# parameters---------------------
waypoint = [40.19373, 140.05923]
altitude = 5
speed = 5
#--------------------------------

async def run():

    main_coroutines = [
        Pixhawk.cycle_flight_mode(),
        Pixhawk.cycle_position_lat_lng(), 
        Pixhawk.cycle_lidar(),
        Pixhawk.cycle_show(),
        Pixhawk.mission_land()
        ]
    
    await Pixhawk.connect()

    await Pixhawk.upload_mission(waypoint, altitude, speed)

    await Pixhawk.health_check()

    await asyncio.gather(*main_coroutines)

    await Pixhawk.arm()

    await Pixhawk.start_mission()


    if __name__ == "__main__":
        Pixhawk = Pixhawk()
        asyncio.run(run())