#timeモジュールをインポート
import time

#RPi.GPIOモジュールをインポート
import RPi.GPIO as GPIO

def fusing():
    # BCM(GPIO番号)で指定する設定
    GPIO.setmode(GPIO.BCM)

    # GPIO26を出力モード設定
    GPIO.setup(26, GPIO.OUT)

    # GPIO26の出力を1にして、LED点灯
    GPIO.output(26, GPIO.OUT)

    # 0.5秒待つ
    time.sleep(5.0)

    # GPIO17の出力を0にして、LED消灯
    GPIO.output(26, GPIO.OUT)

if __name__ == "__main__":
    fusing()