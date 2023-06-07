import asyncio
from mavsdk import System


async def run():
    # Init the drone
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

    # Start the task
    task = cycle_record_log()
    await task

async def cycle_record_log(self) -> None:
        # log_file = self.create_log_file()
        log_dict = {}
        log_dict["mode"] = []
        log_dict["lat_deg"] = []
        log_dict["lng_deg"] = []
        log_dict["abs_alt_m"] = []
        log_dict["rel_alt_m"] = []
        while True:
            # get_position(self)
            async for position in self.drone.telemetry.position():
                self.latitude_deg = position.latitude_deg
                self.longitude_deg = position.longitude_deg
                self.absolute_altitude_m = position.absolute_altitude_m
                self.relative_altitude_m = position.relative_altitude_m
            log_dict["lat_deg"].append(self.latitude_deg)
            log_dict["lng_deg"].append(self.longitude_deg)
            log_dict["abs_alt_m"].append(self.absolute_altitude_m)
            log_dict["rel_alt_m"].append(self.relative_altitude_m)
            print(log_dict)
            await asyncio.sleep(1)

# async def get_position(self) -> None:
#     """get gps position in (lat, lng, abusolute altitude, relative altitude)"""
#     async for position in self.drone.telemetry.position():
#         self.latitude_deg = position.latitude_deg
#         self.longitude_deg = position.longitude_deg
#         self.absolute_altitude_m = position.absolute_altitude_m
#         self.relative_altitude_m = position.relative_altitude_m



if __name__ == "__main__":
    # Start the main function
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())