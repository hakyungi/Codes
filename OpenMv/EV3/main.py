# Untitled - By: JEON - 2023년 5월 11일

# 이프로그램은 OpenMV 카메라에서 물체를 찾아 해당 데이터를 UART로 보내는 프로그램.
#
# 보내는 PACKET의 포맷은 아래와 같음.
#
# PACKET FORMAT (Unsigned Integer 2 Byte는 LSB, MSB순으로 되어 있으며 Little Endian 형태로 전송됨.)
# ---------------------------------------------------------------------------------------------------------
# PACKET HEADER, PACKET SeqNo, ULTRA, TOF, YAW, ROLL, PITCH, Object Count n, OBJECT[0]...OBJECT[n-1], CHECKSUM
# ---------------------------------------------------------------------------------------------------------
# 1. PACKET HEADER  : 2 Byte (0xAA , 0xCC)              -- 0XAA와 0xCC 가 연달아 들어오면 PCKET의 시작임.
# 2. PACKET SeqNo   : Unsigned Integer 1 Byte (0 ~ 127) -- PACKET 송신시마다 1씩 증가함. (수신측에서 이값이 바껴야 새로운 PACKET으로 인식)
# 3. ULTRA  : Unsigned Integer 1 Byte   -- HC-SR04를 이용한 초음파 거리 측정 값.(Format:'<B')
# 4. TOF    : Unsigned Integer 2 Byte   -- VL53L0X를 이용한 TOF 거리 측정 값.(Format:'<H')
# 5. YAW    : Float 4 Byte              -- BNO055를 이용한 YAW 값.(Format:'<f')
# 6. ROLL   : Float 4 Byte              -- BNO055를 이용한 ROLL 값.(Format:'<f')
# 7. PITCH  : Float 4 Byte              -- BNO055를 이용한 PITCH 값.(Format:'<f')
# 8. OBJECT COUNT n  : Unsigned Integer 1 Byte       -- 포함하고 있는 Object의 갯수
# 9. OBJECT[n]      : n개의 Object 정보. 구조는 아래와 같음. (OBJECT당 10 Byte)
#       -----------------------------------------
#       : Object Type, X, Y, Width, Height
#       -----------------------------------------
#       : Object Type   : Unsigned Integer 2 Byte  -- Object의 종류 (예: 1 = Silver Ball, 2 = Black Ball ...)
#       : X             : Unsigned Integer 2 Byte
#       : Y             : Unsigned Integer 2 Byte
#       : R             : Unsigned Integer 2 Byte
#       : MAGNITUDE     : Unsigned Integer 2 Byte
#       -----------------------------------------
# ---------------------------------------------------------------------------------------------------------
# 10. CHECKSUM : PACKET HEADER를 포함한 모든 수신데이터의 Exclusive Or 값. (chksum ^= all recieve data)
# ---------------------------------------------------------------------------------------------------------

import sensor, image, pyb, time
import micropython
import math, struct
from pyb import UART, Pin, Timer
from time import sleep_ms
from pyb import I2C

from HCSR04 import HCSR04
from VL53L0X import VL53L0X
from BNO055 import BNO055

micropython.alloc_emergency_exception_buf(200)

length = {'Int8' : 1, 'uInt8' : 1, 'Int16' : 2, 'uInt16' : 2, 'Int32' : 4, 'uInt32' : 4, 'float' : 4}
format = {'Int8' : '<b', 'uInt8' : '<B', 'Int16' : '<h', 'uInt16' : '<H', 'Int32' : '<l', 'uInt32' : '<L', 'float' : '<f'}

def addData(type, array):
    global PACKET
    if isinstance(array,list):
        array = array[:5]     # max array size is 5 byte
        for element in array:
            PACKET += struct.pack(format[type], element)
    else:
        PACKET += struct.pack(format[type], array)


# -------------------------------------------------------------
# Program
# -------------------------------------------------------------

I2C_J8 = I2C(2, I2C.MASTER, baudrate = 400000)    #Hardware I2C
I2C_J9 = I2C(4, I2C.MASTER, baudrate = 400000)    #Hardware I2C

HCSR04J7 = HCSR04("J7")

TOF = VL53L0X(I2C_J8)
TOF.start()

IMU = BNO055(I2C_J9)

# -------------------------------------------------------------
# UART 1, and baudrate.
# -------------------------------------------------------------
uart = UART(1, 115200)

##Camera Define
sensor.reset()
sensor.set_pixformat(sensor.RGB565) # Format: RGB565.
sensor.set_framesize(sensor.QVGA)   # Size: QQVGA (120x160)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
sensor.set_auto_gain(False)         # Close auto gain (must be turned off for color tracking)
sensor.set_auto_whitebal(False)     # Close white balance (must be turned off for color tracking)

clock = time.clock() # Create a clock object to track the FPS.

PacketSeqNO = 0

while(True):
# -------------------------------------------------------------
# 센서 읽기
# -------------------------------------------------------------
    ultraV = HCSR04J7.distance()
    tofV = TOF.read()
    imuV = IMU.euler()
#    ultraV = 123
#    tofV = 8190
#    imuV = [359, -180, -90]

    print("ULTRA : ", ultraV)
    print("TOF   : ", tofV)
    print("IMU   : ", imuV)

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

    #원 모양을 찾는다.(값들을 잘 조정하여 최적화 시켜야 함.)
    img2 = img
    Circles = img.find_circles(threshold = 5000, x_margin = 20, y_margin = 20, r_margin = 20,
            r_min = 20, r_max = 100, r_step = 10)   #threshold = 3800

    #원모양을 찾았으면
    CircleCount = 0
    SilverCircles = []
    BlackCircles = []

    if Circles:
        for c in Circles:
            CircleCount += 1
            #원 내부의 히스토그램을 얻는다.
            Region = (c.x() - c.r() // 2, c.y() - c.r() // 2 , c.r(), c.r())
            hist = img.get_histogram(roi=Region)

            img.draw_circle(c.x(), c.y(), c.r(), color = (0, 255, 0), fill = False)
#            print('GRAY  : %d' % hist.get_threshold().value())

            # BLACK BALL
            if hist.get_threshold().value() < 10 :
                img.draw_circle(c.x(), c.y(), c.r() // 2, color = (255, 255, 0), fill = True)
                BlackCircles.append(c)
            # SILVER BALL
            elif hist.get_threshold().value() > 15 and hist.get_threshold().value() < 100:
                img.draw_circle(c.x(), c.y(), c.r() // 2, color = (255, 0, 0), fill = True)
                SilverCircles.append(c)

#        print('TOTAL  : %d' %CircleCount)
#        print('SILVER : %d' %len(SilverCircles))
#        print('BLACK  : %d' %len(BlackCircles))

# -------------------------------------------------------------
# #PACKET 만들기
# -------------------------------------------------------------
    #PACKET 만들기
    if len(SilverCircles) > 10:
        SilverCircles = SilverCircles[:10]
    if len(BlackCircles) > 10:
        BlackCircles = BlackCircles[:10]

    ObjectCount = len(SilverCircles)
    ObjectCount += len(BlackCircles)
    PacketSeqNO = (PacketSeqNO + 1) & 0x7F

    PACKET = b''
    addData('uInt8', 0xAA)          # PACKET HEADER 0xAA, 0xCC
    addData('uInt8', 0xCC)
    addData('uInt8', PacketSeqNO)   # Packet Seq No.

    addData('uInt8', ultraV)        # ULTRA : 부호 없는 1 바이트 정수(<B').
    addData('uInt16', tofV)         # TOF : 부호 없는 2 바이트 정수('<H').
    addData('float', imuV[0])       # YAW: 4 바이트 부동 소수('f')
    addData('float', imuV[1])       # ROLL: 4 바이트 부동 소수('f')
    addData('float', imuV[2])       # PITCH: 4 바이트 부동 소수('f')

    addData('uInt8', ObjectCount)     # Object Count

    if ObjectCount:   # 보낼 Object 데이터가 있다면
        for c in SilverCircles:
            buf = [1, c.x() ,c.y() ,c.r(), c.magnitude()]   # Object Type = 1
            addData('uInt16', buf)                          # OBJECT[n]
            print(c)

        for c in BlackCircles:
            buf = [2, c.x() ,c.y() ,c.r(), c.magnitude()]   # Object Type = 2
            addData('uInt16', buf)                          # OBJECT[n]
            print(c)

    chksum = 0
    for b in PACKET:
       chksum ^= b
    addData('uInt8', chksum)        #CHECKSUM

    # 송신하고 송신된 바이트수를 출력
    print("send",uart.write(PACKET))    # Send Packet

    # 이미지에서 찾은 OBJECT의 갯수
    print("Objects %d" % ObjectCount)

    # 송신한 PACKET을 16진수로 출력
    for p in PACKET:
        print("%2X" % p, end = " ")
    print("")

    # 프레임 처리 속도 (Frame Per Seocond(
    print("FPS %f" % clock.fps())
    print("")

#    pyb.delay(10)

    if uart.any() > 0:  # 수신 데이터가 있다면 출력한다.
        print("RECEIVE : ",uart.read(uart.any()))


