import sys

ibis_directory = "/home/pi/ARLISS_IBIS/IB/Library"
sys.path.append(ibis_directory)

import asyncio
from lora import Lora

# parameters---------------------
lora_power_pin = 4
lora_sleep_time = 10
#--------------------------------

async def run():

    lora = Lora(
        lora_power_pin,
        lora_sleep_time
    )
    
    print("Take care that raspi uses pin power suply")
    lora.serial_connect()
    print("sending")
    await lora.write("land judge start")
    await lora.write("1")


if __name__ == "__main__":

    asyncio.run(run())