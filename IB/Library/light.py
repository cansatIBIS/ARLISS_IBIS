import spidev
import sys
from lora import Lora
from logger_lib import logger_info, logger_debug
import time


class Light:
    
    def __init__(self,     
                 light_threshold,
                 stored_timelimit,
                 stored_judge_time,
                 released_timelimit,
                 released_judge_time,
                 lora_power_Pin,
                 deamon_pass = "/home/pi/ARLISS_IBIS/IB/log/Performance_log.txt"):
        
        self.lora = Lora(lora_power_Pin)
        self.light_threshold = light_threshold
        self.stored_timelimit = stored_timelimit
        self.stored_judge_time = stored_judge_time
        self.released_timelimit = released_timelimit
        self.released_judge_time = released_judge_time
        self.deamon_pass = deamon_pass
        self.deamon_file = open(self.deamon_pass)
        self.deamon_log = self.deamon_file.read()
        
    
    def spi_open(self):
    
        self.spi = spidev.SpiDev()     
        self.spi.open(0, 0)                    
        self.spi.max_speed_hz = 1000000 
        
    
    def spi_close(self):
        
        self.spi.close()
        sys.exit()
        
        
    def get_light_val(self):
    
        resp = self.spi.xfer2([0x68, 0x00])                 
        value = ((resp[0] << 8) + resp[1]) & 0x3FF    
        return value
    
    
    async def stored_judge(self):
    
        if "stored judge finish" in self.deamon_log:
            await self.lora.write("skipped stored judge")
            logger_info.info("skipped stored judge")
            return
        
        else:
            await self.lora.write("stored judge start")
            logger_info.info("######################\n# stored judge start #\n######################")

            start_time = time.perf_counter()
            duration_start_time = time.perf_counter()
            is_continue = False

            while True:


                light_val = self.get_light_val()
                time_stamp = time.perf_counter() - duration_start_time
                logger_info.info("{:5.1f}| 光センサ:{:>3d}, 継続:{}".format(time_stamp, light_val, is_continue))

                if is_continue:
                    if light_val >= self.light_threshold:
                        is_continue = False
                        continue

                    duration_time = time.perf_counter() - duration_start_time

                    if duration_time > self.stored_judge_time:
                        logger_info.info("-- Light Judge")
                        break
                
                elif light_val < self.light_threshold:
                    is_continue = True
                    duration_start_time = time.perf_counter()
                
                elapsed_time = time.perf_counter() - start_time

                if elapsed_time > self.stored_timelimit:
                    logger_info.info("-- Timer Judge")
                    break

            await self.lora.write("stored judge finish")
            logger_info.info("#######################\n# stored judge finish #\n#######################")


    async def released_judge(self):
        
        if "released judge finish" in self.deamon_log:
            await self.lora.write("skipped released judge")
            logger_info.info("skipped released judge")
            return
        
        else:
            await self.lora.write("released judge start")
            logger_info.info("########################\n# released judge start #\n########################")

            start_time = time.perf_counter()
            duration_start_time = time.perf_counter()
            is_continue = False


            while True:

                light_val = self.get_light_val()
                time_stamp = time.perf_counter() - duration_start_time
                logger_info.info("{:5.1f}| 光センサ:{:>3d}, 継続:{}".format(time_stamp, light_val, is_continue))

                if is_continue:
                    if light_val <= self.light_threshold:
                        is_continue = False
                        continue

                    duration_time = time.perf_counter() - duration_start_time

                    if duration_time > self.released_judge_time:
                        logger_info.info("-- Light Judge")
                        break
                
                elif light_val > self.light_threshold:
                    is_continue = True
                    duration_start_time = time.perf_counter()
                
                elapsed_time = time.perf_counter() - start_time

                if elapsed_time > self.released_timelimit:
                    logger_info.info("-- Timer Judge")
                    break

            await self.lora.write("released judge finish")
            logger_info.info("#########################\n# released judge finish #\n#########################")
