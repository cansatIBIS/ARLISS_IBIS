#timeモジュールをインポート
import time

#RPi.GPIOモジュールをインポート
import RPi.GPIO as GPIO

def fusing():
    # BCM(GPIO番号)で指定する設定
    try:
        print("-- Start")
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)

        # GPIO26を出力モード設定
        GPIO.setup(17, GPIO.OUT)

        # GPIO26の出力を1にして、LED点灯
        GPIO.output(17, 1)
        print("-- Fusing")

        # 0.5秒待つ
        time.sleep(5.0)
        print("-- Fused! Please Fly")

        # GPIO17の出力を0にして、LED消灯
        GPIO.output(17, 0)
        
        GPIO.cleanup()
        
    
    except KeyboardInterrupt:
        # GPIO設定クリア
        GPIO.cleanup()

if __name__ == "__main__":
    fusing()