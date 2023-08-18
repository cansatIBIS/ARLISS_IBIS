# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO

SWITCH_PIN = 4 # ピン番号

GPIO.setmode(GPIO.BCM)
GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # プルアップの場合
# GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # プルダウンの場合

result = GPIO.input(SWITCH_PIN) # ピンの値を読み取る(HIGH or LOWの1 or 0)
print(result)