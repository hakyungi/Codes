# Untitled - By: JEON - 금 12 23 2022

# Find Circles Example
#
# This example shows off how to find circles in the image using the Hough
# Transform. https://en.wikipedia.org/wiki/Circle_Hough_Transform
#
# Note that the find_circles() method will only find circles which are completely
# inside of the image. Circles which go outside of the image/roi are ignored...
#----------------------------------------------------------------------------------
# 보내는 데이터의 정의
# 데이터는 DataToSend[] 배열에 담겨 전송시킨다.
# 8개의 데이터는 아래의 포맷을 갖는다.
#----------------------------------------------------------------------------------
# DataToSend[0] : Packet Number (매전송시 1씩 증가한다).
# DataToSend[1] : DATA TYPE
#                   1 : TOF VALUE
#                   2 : TEMP VALUE
#                   3 : IMU VALUE
#                   4 : ?
#                   5 : ?
# DataToSend[2] : 첫번째 값 (TOF1, TEMP 1 주변, YAW, CX)
# DataToSend[3] : 두번째 값 (TOF2, TEMP 1 대상, ROLL, CY)
# DataToSend[4] : 세번째 값 (TOF3, TEMP 2 주변, PITCH, W)
# DataToSend[5] : 네번째 값 (TOF4, TEMP 2 대상, dummy, H)
# DataToSend[6] : 다섯번째 값 (dummy : 사용안함)
# DataToSend[7] : 여섯번째 값 (dummy : 사용안함)
#----------------------------------------------------------------------------------

waitForSending = 5 # 데이터가 전송될때까지 기다려 줄 시간(ms)

import sensor, image, pyb, time
import gc,utime
import micropython
import LPF2
from machine import Pin
from machine import SoftI2C
from time import sleep_ms

from VL53L0X import VL53L0X     # TOF 거리측정 센서
from BNO055 import BNO055       # 9DOF IMU 센서
from MLX90614 import MLX90614   # 비접촉 온도 센서

micropython.alloc_emergency_exception_buf(200)

##LUMP Define
modes = [LPF2.mode('UART_READ8',size = 8, type = LPF2.DATA16, format = '3.0'),]

DataToSend = [1, 3, 5, 7, 9, 11, 13, 15] #X, Y, W, H, ID, state, 0, 0
max_idx = -1

lpf2 = LPF2.EV3_LPF2(1, 'P1', 'P0', modes, 85, timer = 4, freq = 50) #ID 85 to match our block
lpf2.initialize()

##Camera Define

ball_threshold   = (   15,   30,  0,   40,   0,   40) # Middle L, A, B values.

sensor.reset()
sensor.set_pixformat(sensor.RGB565) # Format: RGB565.
sensor.set_framesize(sensor.QVGA) # Size: QQVGA (120x160)
sensor.skip_frames(time = 2000) # Wait for settings take effect.
sensor.set_auto_gain(False) # Close auto gain (must be turned off for color tracking)
sensor.set_auto_whitebal(False) # Close white balance (must be turned off for color tracking)
clock = time.clock() # Create a clock object to track the FPS.

PacketCount = 0

#pyb.LED(1).on()
#pyb.LED(2).on()
#pyb.LED(3).on()
#pyb.LED(4).on()

#---------------------------------------------------------------------
# Software I2C 채널 (온도센서와의 최대 통신속도는 100KHz 임)
#---------------------------------------------------------------------
I2C_J7 = SoftI2C(scl = 'P2', sda = 'P3', freq = 100000, timeout=50000) #Soft I2C
I2C_J8 = SoftI2C(scl = 'P4', sda = 'P5', freq = 100000, timeout=50000) #Soft I2C
I2C_J9 = SoftI2C(scl = 'P7', sda = 'P8', freq = 400000, timeout=50000) #Soft I2C
I2C_J15 = SoftI2C(scl = 'P6', sda = 'P9', freq = 400000, timeout=50000) #Soft I2C
#---------------------------------------------------------------------

TOF1 = VL53L0X(I2C_J7)
TOF2 = VL53L0X(I2C_J8)
TOF3 = VL53L0X(I2C_J9)
#TOF4 = VL53L0X(I2C_J15)
TOF1.start()
TOF2.start()
TOF3.start()
#TOF4.start()

TEMP1 = MLX90614(I2C_J7)
TEMP2 = MLX90614(I2C_J8)

IMU = BNO055(I2C_J9)

def send_SensorValue():
    global PacketCount

    while not lpf2.sent:
        pass

    PacketCount += 1
    #-------------------------------------------------------------
    # 센서 읽기
    #-------------------------------------------------------------
    tof1V = TOF1.read()
    tof2V = TOF2.read()
    tof3V = TOF3.read()
    #tof4V = TOF4.read()

    temp1E = int(TEMP1.getEnvCelsius() * 10)
    sleep_ms(2)
    temp1O = int(TEMP1.getObjCelsius() * 10)
    sleep_ms(2)

    temp2E = int(TEMP2.getEnvCelsius() * 10)
    sleep_ms(2)
    temp2O = int(TEMP2.getObjCelsius() * 10)
    sleep_ms(2)

    imuV = IMU.euler()

    #-------------------------------------------------------------
    # PACKET 만들어 보내기
    #-------------------------------------------------------------
    DataToSend = [PacketCount, 1, tof1V, tof2V, tof3V, temp1O, temp2O, int(imuV[0]*10)]
    lpf2.load_payload('Int16',DataToSend)

while(True):
    if not lpf2.connected:
        print(lpf2.modes)
        lpf2.sendTimer.deinit()
        utime.sleep_ms(50)
        lpf2.initialize()
    else:
        send_SensorValue()

        clock.tick()
        img = sensor.snapshot().lens_corr(1.8)

        # Circle objects have four values: x, y, r (radius), and magnitude. The
        # magnitude is the strength of the detection of the circle. Higher is
        # better...

        # `threshold` controls how many circles are found. Increase its value
        # to decrease the number of circles detected...

        # `x_margin`, `y_margin`, and `r_margin` control the merging of similar
        # circles in the x, y, and r (radius) directions.

        # r_min, r_max, and r_step control what radiuses of circles are tested.
        # Shrinking the number of tested circle radiuses yields a big performance boost.

    #    for c in img.find_circles(threshold = 2000, x_margin = 10, y_margin = 10, r_margin = 10,
    #            r_min = 2, r_max = 100, r_step = 2):
    #        img.draw_circle(c.x(), c.y(), c.r(), color = (255, 0, 0))
    #        print(c)


        circleCount = 0

        #원 모양을 찾는다.(값들을 잘 조정하여 최적화 시켜야 함.)
        Circles = img.find_circles(threshold = 3800, x_margin = 20, y_margin = 20, r_margin = 20,
                r_min = 20, r_max = 100, r_step = 10)

        #원모양을 찾았으면
        BlackCircle = ''
        SilverCircle = ''
        if Circles:
            circlecount1 = len(Circles)
            SilverCircles = []
            BlackCircles = []

            for c in Circles:
                circleCount = circleCount + 1

                #원 내부의 히스토그램을 얻는다.
                Region = (c.x() - c.r() //2, c.y() - c.r() // 2 , c.r(), c.r())
                hist = img.get_histogram(roi=Region)

                img.draw_circle(c.x(), c.y(), c.r(), color = (0, 255, 0), fill = False)
                #print('GRAY  : %d' % hist.get_threshold().value())

                if hist.get_threshold().value() < 10 :
                    img.draw_circle(c.x(), c.y(), c.r() // 2, color = (255, 255, 0), fill = True)
                    BlackCircles.append(c)
                    if not BlackCircle:
                        BlackCircle = c
                    else:
                        if c.r() > BlackCircle.r():
                            BlackCircle = c
                elif hist.get_threshold().value() > 15 and hist.get_threshold().value() < 100:
                    PacketCount+=1
                    img.draw_circle(c.x(), c.y(), c.r() // 2, color = (255, 0, 0), fill = True)
                    SilverCircles.append(c)
                    if not SilverCircle:
                        SilverCircle = c
                    else:
                        if c.r() > SilverCircle.r():
                            SilverCircle = c

            while not lpf2.sent:
                pass

            PacketCount += 1
            if BlackCircle:
                DataToSend = [PacketCount, 2, BlackCircle.x(), BlackCircle.y(), BlackCircle.r(), BlackCircle.r(), 0, 0]
            else:
                DataToSend = [PacketCount, 2, 0, 0, 0, 0, 0, 0]
            lpf2.load_payload('Int16',DataToSend)

            while not lpf2.sent:
                pass

            PacketCount += 1
            if SilverCircle:
                DataToSend = [PacketCount, 3, SilverCircle.x(), SilverCircle.y(), SilverCircle.r(), SilverCircle.r(), 0, 0]
            else:
                DataToSend = [PacketCount, 3, 0, 0, 0, 0, 0, 0]
            lpf2.load_payload('Int16',DataToSend)

            #print('TOTAL  : %d' %circleCount)
            #print('SILVER : %d' %len(SilverCircles))
            #print('BLACK  : %d' %len(BlackCircles))


        #print("FPS %f" % clock.fps())
        #print("Circles %d" % circleCount)
        pyb.delay(100)

