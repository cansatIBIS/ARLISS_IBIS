from typing import Any
import spidev
from logger_lib import logger_info
import time


class Light:
    
    def __init__(self,     
                 light_threshold,
                 stored_timelimit,
                 stored_judge_time,
                 released_timelimit,
                 released_judge_time,
                 lora,
                 deamon_pass = "/home/pi/ARLISS_IBIS/IB/log/Performance_log.txt"):
        
        self.lora = lora
        
        self.light_threshold = light_threshold
        self.stored_timelimit = stored_timelimit
        self.stored_judge_time = stored_judge_time
        self.released_timelimit = released_timelimit
        self.released_judge_time = released_judge_time
        self.deamon_pass = deamon_pass
        self.deamon_file = open(self.deamon_pass)
        self.deamon_log = self.deamon_file.read()
        self.spi_open()
        
        logger_info.info("Light initialized")
        
    
    def spi_open(self):
    
        self.spi = spidev.SpiDev()     
        self.spi.open(0, 0)                    
        self.spi.max_speed_hz = 1000000 
        
    
    def spi_close(self):
        
        self.spi.close()
        
        
    def get_light_val(self):
    
        resp = self.spi.xfer2([0x68, 0x00])                 
        value = ((resp[0] << 8) + resp[1]) & 0x3FF  
        if value == 0:
            value = float("nan")
        return value
    
    
    async def stored_judge(self):
    
        if "stored judge finish" in self.deamon_log:
            logger_info.info("skipped stored judge")
            return
        
        else:
            logger_info.info("#################### stored judge start ####################")

            start_time = time.perf_counter()
            duration_start_time = time.perf_counter()
            is_continue = False
            pre_time_stamp = 0

            while True:


                light_val = self.get_light_val()
                time_stamp = time.perf_counter() - duration_start_time
                if abs(pre_time_stamp - time_stamp) > 0.4:
                    pre_time_stamp = time_stamp
                    logger_info.info("{:5.1f}| 光センサ:{:>3}, 継続:{}".format(time_stamp, light_val, is_continue))
                    
                
                if light_val < self.light_threshold:
                    pass
                
                else:
                    is_continue = False
                    continue

                if is_continue:

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

            logger_info.info("#################### stored judge finish ####################")


    async def released_judge(self):
        
        if "released judge finish" in self.deamon_log:
            logger_info.info("skipped released judge")
            return
        
        else:
            logger_info.info("#################### released judge start ####################")

            start_time = time.perf_counter()
            duration_start_time = time.perf_counter()
            is_continue = False
            pre_time_stamp = 0


            while True:

                light_val = self.get_light_val()
                time_stamp = time.perf_counter() - duration_start_time
                if abs(pre_time_stamp - time_stamp) > 0.4:
                    pre_time_stamp = time_stamp
                    logger_info.info("{:5.1f}| 光センサ:{:>3d}, 継続:{}".format(time_stamp, light_val, is_continue))
                    
                if is_continue:
                    
                    if light_val > self.light_threshold:
                        pass
                    
                    else:
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

            logger_info.info("#################### released judge finish ####################")


    def __del__(self):
        
        self.spi_close()
