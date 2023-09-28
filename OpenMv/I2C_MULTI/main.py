# Untitled - By: JEON - 금 9 22 2023
from micropython import const
import ustruct
import time
import utime
import pyb
from machine import SoftI2C
from pyb import I2C

from bno055 import BNO055
from VL53L0X import VL53L0X
from MLX90614 import MLX90614

#---------------------------------------------------------------------
# 사용할 I2C 채널을 선택하세요.
#---------------------------------------------------------------------
#---------------------------------------------------------------------
# Software I2C 채널
#---------------------------------------------------------------------
#I2C_J7 = SoftI2C(scl = 'P2', sda = 'P3', freq = 100000, timeout=50000) #Soft I2C
#I2C_J8 = SoftI2C(scl = 'P4', sda = 'P5', freq = 100000, timeout=50000) #Soft I2C
#I2C_J9 = SoftI2C(scl = 'P7', sda = 'P8', freq = 100000, timeout=50000) #Soft I2C

#---------------------------------------------------------------------
# Hardware I2C 채널
#---------------------------------------------------------------------
I2C_J8 = I2C(2, I2C.MASTER, baudrate = 100000)    #Hardware I2C
#I2C_J9 = I2C(4, I2C.MASTER, baudrate = 100000)    #Hardware I2C
#---------------------------------------------------------------------


IMU = BNO055(I2C_J8, address=0x28)
GY906 = MLX90614(I2C_J8)
TOF = VL53L0X(I2C_J8)
TOF.start()

clock = time.clock()
clock.tick()

while True:
    print("GY906 대상 온도  %s *C"% GY906.getObjCelsius())        #print celsius of Object
    print("GY906 주변 온도  %s *C"% GY906.getEnvCelsius())        #print celsius of Ambient
    print("")
    print("TOF 거리 %smm "%TOF.read())
    print("IMU ", IMU.euler())
    print("")
    clock.tick()
    time.sleep_ms(500)


