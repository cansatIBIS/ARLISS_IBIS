import asyncio
from mavsdk import System


async def run():
    # Init the drone
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():

        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    # Start the task
    asyncio.ensure_future(create_position_dict(drone))

    while True:
        await asyncio.sleep(1)

async def create_position_dict(drone) -> None:
        # log_file = self.create_log_file()
        log_dict = {}
        log_dict["lat_deg"] = []
        log_dict["lng_deg"] = []
        log_dict["abs_alt_m"] = []
        log_dict["rel_alt_m"] = []
        # get_position(self)
        async for position in drone.telemetry.position():
            latitude_deg = position.latitude_deg
            longitude_deg = position.longitude_deg
            absolute_altitude_m = position.absolute_altitude_m
            relative_altitude_m = position.relative_altitude_m
        log_dict["lat_deg"].append(latitude_deg)
        log_dict["lng_deg"].append(longitude_deg)
        log_dict["abs_alt_m"].append(absolute_altitude_m)
        log_dict["rel_alt_m"].append(relative_altitude_m)
        print(log_dict)


# async def get_position(self) -> None:
#     """get gps position in (lat, lng, abusolute altitude, relative altitude)"""
#     async for position in self.drone.telemetry.position():
#         self.latitude_deg = position.latitude_deg
#         self.longitude_deg = position.longitude_deg
#         self.absolute_altitude_m = position.absolute_altitude_m
#         self.relative_altitude_m = position.relative_altitude_m



if __name__ == "__main__":
    # Start the main function
    asyncio.run(run())