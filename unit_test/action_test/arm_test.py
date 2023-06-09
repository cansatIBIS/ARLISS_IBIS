import asyncio
from Library.Ibis import Ibis


async def run():

    drone = Ibis()
    drone.connect()
    drone.arm()

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
