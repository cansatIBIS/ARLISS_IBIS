import sys
import asyncio

ibis_directory = "/home/pi/ARLISS_IBIS/IB/Library"
sys.path.append(ibis_directory)

from lora import Lora


lora_sleep_time = 3 
lora_power_pin = 4


async def run():
  
  lora = Lora(lora_power_pin,
              lora_sleep_time)
  lora.write("land judge start")
  


if __name__ == "__main__":
    
    asyncio.run(run())