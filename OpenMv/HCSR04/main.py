# 이프로그램은 OpenMV 카메라에서 HC-SR04 초음파센서로 거리를 측정.

from time import sleep_ms
from HCSR04 import HCSR04

#HCSR04J7 = HCSR04("J7")
#HCSR04J8 = HCSR04("J8")
HCSR04J9 = HCSR04("J9")

while True:
#   print('distance J7 = ',HCSR04J7.distance())
#   print('distance J8 = ',HCSR04J8.distance())
   print('distance J9 = ',HCSR04J9.distance())
   sleep_ms(0)
