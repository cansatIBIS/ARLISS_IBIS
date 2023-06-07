import asyncio
from mavsdk import System
from mavsdk.action import OrbitYawBehavior

orbit_hight = 5
orbit_radius = 5

def get_xy_position(drone):
    position = drone.telemetry.position()
    return position

async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    position = await drone.telemetry.position().__aiter__().__anext__()
    orbit_height += position.absolute_altitude_m
    yaw_behavior = OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER
    
    xy_coo = get_xy_position()

    print("-- Arming")
    await drone.action.arm()

    print("--- Taking Off")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    print('Do orbit at {}m height from the ground'.format(orbit_hight))
    await drone.action.do_orbit(radius_m=orbit_radius,
                                velocity_ms=2,
                                yaw_behavior=yaw_behavior,
                                latitude_deg=xy_coo[0],
                                longitude_deg=xy_coo[1],
                                absolute_altitude_m=orbit_height)
    await asyncio.sleep(60)

    await drone.action.return_to_launch()
    print("--- Landing")

if __name__ == "__main__":
    asyncio.run(run())