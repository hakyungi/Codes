from machine import SoftI2C
from pyb import I2C
from time import sleep_ms
from TCS3472 import TCS3472

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
#I2C_J8 = I2C(2, I2C.MASTER, baudrate = 100000)    #Hardware I2C
I2C_J9 = I2C(4, I2C.MASTER, baudrate = 400000)    #Hardware I2C
#---------------------------------------------------------------------

t = TCS3472(I2C_J9)
sleep_ms(10)

while True:
    r, g, b = t.rgb()
    l = t.brightness()
    print("R:%d"%r , "G:%d"%g , "B:%d"%b , "C:%d"%l)
    sleep_ms(200)
