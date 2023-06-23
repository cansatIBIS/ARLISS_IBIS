import asyncio
from mavsdk import System
import time
import datetime
import csv

async def run():
    start = time.time()
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    # center_lat_deg_list = []
    # center_lng_deg_list = []
    # async for _ in range(10):
    #     async for position in drone.telemetry.position():
    #         lat_deg = position.latitude_deg
    #         lng_deg = position.longitude_deg
    #         center_lat_deg_list.append(lat_deg)
    #         center_lng_deg_list.append(lng_deg)
    #         break
    # print("got gps")

    # center_lat_deg_ave = sum(center_lat_deg_list)/10
    # center_lng_deg_ave = sum(center_lng_deg_list)/10
    
    # center = [center_lat_deg_ave, center_lng_deg_ave]
    # print(f"中心の緯度={center[0]}")
    # print(f"中心の経度={center[1]}")
    
    init_abs_alt=0
    async for position in drone.telemetry.position():
        init_abs_alt=position.absolute_altitude_m
        print(init_abs_alt)
        break

    rel_alt_lst=[]
    lidar_lst=[]
    velocity_lst_alt=[]
    velocity_lst_lidar=[]
    latitude_deg_lst=[]
    longitude_deg_lst=[]
    while True:
        pretime = time.time()
        async for position in drone.telemetry.position():
            abs_alt = position.absolute_altitude_m
            rel_alt = -(abs_alt - init_abs_alt)
            rel_alt_lst.append(rel_alt)
            break
        async for distance in drone.telemetry.distance_sensor():
            lidar_height = distance.current_distance_m
            lidar_lst.append(lidar_height)
            break
        posttime = time.time()
        difference = posttime - pretime

        velocity_lst_alt.append(rel_alt / difference)
        velocity_lst_lidar.append(lidar_height / difference)
        
        async for position in drone.telemetry.position():
            latitude_deg_lst.append(position.latitude_deg)
            longitude_deg_lst.append(position.longitude_deg)
            break
            # position_lst.append("lat_deg:{} lng_deg:{} abs_alt_m:{} rel_alt_m:{}".format(position.latitude_deg,position.longitude_deg,position.absolute_altitude_m,position.relative_altitude_m))

        print(posttime-start)
        if posttime-start>60:
            break

    dt_now = datetime.datetime.now()
    with open(f"/home/pi/ARLISS_IBIS/log/log_csv/velocity_test {dt_now}.csv","w") as file:
        writer = csv.writer(file)
        writer.writerow(rel_alt_lst)
        writer.writerow(lidar_lst)
        writer.writerow(velocity_lst_alt)
        writer.writerow(velocity_lst_alt)
        writer.writerow(latitude_deg_lst)
        writer.writerow(longitude_deg_lst)

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())