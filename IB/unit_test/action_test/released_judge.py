import spidev               
import time                         
import sys                          

light_threshold = 400
# 連続して値を読み込む
def get_light_val():
    resp = spi.xfer2([0x68, 0x00])                 
    value = ((resp[0] << 8) + resp[1]) & 0x3FF    
    return value

def released_judge():
    print("########################\n# released judge start #\n########################")

    # 関数の開始時間
    start_time = time.perf_counter()
    # 光の継続時間
    duration_start_time = time.perf_counter()
    is_continue = False


    while True:

        light_val = get_light_val()
        time_stamp = time.perf_counter() - duration_start_time
        print("{:5.1f}| 光センサ:{:>3d}, 継続:{}".format(time_stamp, light_val, is_continue))

        if is_continue:
            # 光が途切れていた場合、やり直し
            if light_val <= light_threshold:
                is_continue = False
                continue

            # 光の継続時間
            duration_time = time.perf_counter() - duration_start_time

            if duration_time > 30:
                print("released judge case 1")
                break
        
        elif light_val > light_threshold:
            is_continue = True
            duration_start_time = time.perf_counter()
        
        elapsed_time = time.perf_counter() - start_time

        if elapsed_time > 120:
            print("released judge case 2")
            break

    print("#########################\n# released judge finish #\n#########################")
        

if __name__ == "__main__":
    # SPI
    spi = spidev.SpiDev()     
    spi.open(0, 0)                    
    spi.max_speed_hz = 1000000 

    released_judge()

    spi.close()
    sys.exit()