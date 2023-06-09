import asyncio
from Library.Ibis import Ibis


async def run():

    drone = Ibis()
    drone.connect()
    drone.arm()

if __name__ == "__main__":
    asyncio.run(run())
