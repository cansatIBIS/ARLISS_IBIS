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
x_list=[]
y_list=[]
z_list=[]
r_deg=[]
p_deg=[]
y_deg=[]
forward_a=[]
right_a=[]
down_a=[]
acc=[]

async def run():
    start = time.time()
    # Init the drone
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    print("connected to drone")
    async for o in drone.telemetry.odometry():
        pb = o.position_body
        x=pb.x_m
        y=pb.y_m
        z=pb.z_m
        break
    # async for d in drone.telemetry.attitude_euler():
    #     roll=d.roll_deg
    #     pitch=d.pitch_deg
    #     yaw=d.yaw_deg
    #     break


    while True:
        async for o in drone.telemetry.odometry():
            pb = o.position_body
            x_list.append(pb.x_m - x)
            y_list.append(pb.y_m - y)
            z_list.append(pb.z_m - z)
            # print(f"position_body:{pb}")
            break
        async for d in drone.telemetry.attitude_euler():
            # r_deg.append(d.roll_deg - roll)
            # p_deg.append(d.pitch_deg - pitch)
            # y_deg.append(d.yaw_deg - yaw)
            r_deg.append(d.roll_deg)
            p_deg.append(d.pitch_deg)
            y_deg.append(d.yaw_deg)
            break
        async for v in drone.telemetry.velocity_ned():
            vx_list.append(v.north_m_s)
            vy_list.append(v.east_m_s)
            vz_list.append(v.down_m_s)
            v_list.append(m.sqrt((v.north_m_s)**2+(v.east_m_s)**2+(v.down_m_s)**2))
            # print(f"velocity:{v.down_m_s}")
            break
        async for i in drone.telemetry.imu():
            a = i.acceleration_frd
            forward_a.append(a.forward_m_s2)
            right_a.append(a.right_m_s2)
            down_a.append(a.down_m_s2)
            print(forward_a)
            print(right_a)
            print(down_a)
            acc.append(m.sqrt(forward_a**2+right_a**2+down_a**2))
            break


        now = time.time()
        print(now-start)
        if now-start>15:
            break



    dt_now = datetime.datetime.now()
    with open(f"/home/pi/ARLISS_IBIS/log/log_csv/velocity_check_ver2 {dt_now}.csv","w") as file:
        writer = csv.writer(file)
        writer.writerow(vx_list)
        writer.writerow(vy_list)
        writer.writerow(vz_list)
        writer.writerow(v_list)
        writer.writerow(x_list)
        writer.writerow(y_list)
        writer.writerow(z_list)
        writer.writerow(r_deg)
        writer.writerow(p_deg)
        writer.writerow(y_deg)
        writer.writerow(forward_a)
        writer.writerow(right_a)
        writer.writerow(down_a)
        writer.writerow(acc)



    


if __name__ == "__main__":
    # Start the main function
    asyncio.run(run())