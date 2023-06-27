#!/usr/bin/env python3

import asyncio
import csv
import datetime
from mavsdk import System
from logger import logger_info, logger_debug
import atexit


class Ibis(object):
    def __init__(self):
        self.drone = System()
    
    async def connect(self):
        await self.drone.connect(system_address="serial:///dev/ttyACM0:115200")
        print("Waiting for drone to connect...")
        logger_info.info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone!")
                logger_info.info("-- Connected to drone!")
                break

if __name__ == "__main__":
    test = Ibis()
    asyncio.run(test.connect())
