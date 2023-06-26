#!/usr/bin/env python3

import asyncio
from mavsdk import System
import time
import datetime
import csv
import math as m

vx_list=[]
vy_list=[]
vz_list=[]
v_list=[]
pb_list = []
async def run():
    start = time.time()
    # Init the drone
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    print("connected to drone")


    while True:
        async for v in drone.telemetry.velocity_ned():
            vx_list.append(v.north_m_s)
            vy_list.append(v.east_m_s)
            vz_list.append(v.down_m_s)
            v_list.append(m.sqrt((v.north_m_s)**2+(v.east_m_s)**2+(v.down_m_s)**2))
            # print(f"velocity:{v.down_m_s}")
            break
        async for o in drone.telemetry.odometry():
            pb = o.position_body
            pb_list.append(pb)
            print(f"position_body:{pb}")
            break


        now = time.time()
        print(now-start)
        if now-start>30:
            break



    dt_now = datetime.datetime.now()
    with open(f"/home/pi/ARLISS_IBIS/log/log_csv/velocity_check_ver2 {dt_now}.csv","w") as file:
        writer = csv.writer(file)
        writer.writerow(vx_list)
        writer.writerow(vy_list)
        writer.writerow(vz_list)
        writer.writerow(v_list)



    


if __name__ == "__main__":
    # Start the main function
    asyncio.run(run())