import asyncio
from Library.pixhawk import Pixhawk

# parameters---------------------
fuse_PIN = 0
wait_time = 0
lora_sleep_time = 0
fuse_time = 0
land_timelimit = 0
health_continuous_count = 10
waypoint_lat = 40.19373
waypoint_lng = 140.05923
waypoint_alt = 5
mission_speed = 5
image_navigation_timeout = 6 * 60
#--------------------------------

async def run():

    pixhawk = Pixhawk(
                 fuse_PIN,
                 wait_time,
                 lora_sleep_time, 
                 fuse_time,
                 land_timelimit,
                 health_continuous_count,
                 waypoint_lat,
                 waypoint_lng,
                 waypoint_alt,
                 mission_speed,
                 image_navigation_timeout,
                 )

    # main_coroutines = [
    #     pixhawk.cycle_flight_mode(),
    #     pixhawk.cycle_position_lat_lng(), 
    #     pixhawk.cycle_lidar(),
    #     pixhawk.cycle_show(),
    #     pixhawk.mission_land()
    #     ]
    
    await pixhawk.connect()

    await pixhawk.upload_mission()

    await pixhawk.hold()

    await pixhawk.health_check()

    await pixhawk.arm()

    await pixhawk.start_mission()

    await pixhawk.gather_main_coroutines

if __name__ == "__main__":

    asyncio.run(run())