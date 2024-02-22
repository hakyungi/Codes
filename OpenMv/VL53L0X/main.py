import time

from machine import SoftI2C
from pyb import I2C

from VL53L0X import VL53L0X


#---------------------------------------------------------------------
# 사용할 I2C 채널을 선택하세요.
#---------------------------------------------------------------------
#---------------------------------------------------------------------
# Software I2C 채널
#---------------------------------------------------------------------
I2C_J7 = SoftI2C(scl = 'P2', sda = 'P3', freq = 400000, timeout=50000) #Soft I2C
I2C_J8 = SoftI2C(scl = 'P4', sda = 'P5', freq = 400000, timeout=50000) #Soft I2C
I2C_J9 = SoftI2C(scl = 'P7', sda = 'P8', freq = 400000, timeout=50000) #Soft I2C
I2C_J15 = SoftI2C(scl = 'P6', sda = 'P9', freq = 400000, timeout=50000) #Soft I2C

#---------------------------------------------------------------------
# Hardware I2C 채널
#---------------------------------------------------------------------
#I2C_J8 = I2C(2, I2C.MASTER, baudrate = 400000)    #Hardware I2C
#I2C_J9 = I2C(4, I2C.MASTER, baudrate = 400000)    #Hardware I2C
#---------------------------------------------------------------------

TOF = VL53L0X(I2C_J15)

TOF.start()

clock = time.clock()
clock.tick()

while True:
    print(TOF.read(),clock.avg())
    clock.tick()
