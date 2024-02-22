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
# 3. TOF1   : Unsigned Integer 2 Byte   -- VL53L0X를 이용한 TOF 거리 측정 값.(Format:'<H')
# 4. TOF2   : Unsigned Integer 2 Byte   -- VL53L0X를 이용한 TOF 거리 측정 값.(Format:'<H')
# 5. TOF3   : Unsigned Integer 2 Byte   -- VL53L0X를 이용한 TOF 거리 측정 값.(Format:'<H')
# 6. YAW    : Float 4 Byte              -- BNO055를 이용한 YAW 값.(Format:'<f')
# 7. ROLL   : Float 4 Byte              -- BNO055를 이용한 ROLL 값.(Format:'<f')
# 8. PITCH  : Float 4 Byte              -- BNO055를 이용한 PITCH 값.(Format:'<f')
# 9. OBJECT COUNT n  : Unsigned Integer 1 Byte       -- 포함하고 있는 Object의 갯수
# 10. OBJECT[n]      : n개의 Object 정보. 구조는 아래와 같음. (OBJECT당 10 Byte)
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

import sensor, image
import struct
from pyb import UART
from time import sleep_ms, clock
from machine import SoftI2C

from VL53L0X import VL53L0X #TOF 거리측정 센서
from BNO055 import BNO055   # 9DOF IMU 센서

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

#---------------------------------------------------------------------
# Software I2C 채널
#---------------------------------------------------------------------
I2C_J7 = SoftI2C(scl = 'P2', sda = 'P3', freq = 400000, timeout=50000) #Soft I2C
I2C_J8 = SoftI2C(scl = 'P4', sda = 'P5', freq = 400000, timeout=50000) #Soft I2C
I2C_J9 = SoftI2C(scl = 'P7', sda = 'P8', freq = 400000, timeout=50000) #Soft I2C
I2C_J15 = SoftI2C(scl = 'P6', sda = 'P9', freq = 400000, timeout=50000) #Soft I2C
#---------------------------------------------------------------------

TOF1 = VL53L0X(I2C_J7)
TOF2 = VL53L0X(I2C_J8)
TOF3 = VL53L0X(I2C_J9)
TOF1.start()
TOF2.start()
TOF3.start()

IMU = BNO055(I2C_J9)

# -------------------------------------------------------------
# UART 1, and baudrate.
# -------------------------------------------------------------
uart = UART(1, 115200)

##Camera Define
sensor.reset()
sensor.set_pixformat(sensor.RGB565) # Format: RGB565.
sensor.set_framesize(sensor.QVGA)   # Size: QVGA(320x240), QQVGA (160x120)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
sensor.set_auto_gain(False)         # Close auto gain (must be turned off for color tracking)
sensor.set_auto_whitebal(False)     # Close white balance (must be turned off for color tracking)

clock = clock() # Create a clock object to track the FPS.

PacketSeqNO = 0 # Packet Sequence Number 매 패켓마다 1씩 증가 (0 ~ 127)

# 메인 루프
while(True):
    #-------------------------------------------------------------
    # 센서 읽기
    #-------------------------------------------------------------
    tof1V = TOF1.read()
    tof2V = TOF2.read()
    tof3V = TOF3.read()

    imuV = IMU.euler()

    print("TOF1 : ", TOF1.read())
    print("TOF2 : ", TOF2.read())
    print("TOF3 : ", TOF3.read())
    print("IMU   : ", IMU.euler())

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

    #-------------------------------------------------------------
    # PACKET 만들기
    #-------------------------------------------------------------
    if len(SilverCircles) > 10: #최대 10개까지만
        SilverCircles = SilverCircles[:10]
    if len(BlackCircles) > 10:  #최대 10개까지만
        BlackCircles = BlackCircles[:10]

    ObjectCount = len(SilverCircles)
    ObjectCount += len(BlackCircles)
    PacketSeqNO = (PacketSeqNO + 1) & 0x7F  # 0 ~ 127

    PACKET = b''
    #--------------------------------------------------------------------
    # Packet Header
    #--------------------------------------------------------------------
    addData('uInt8', 0xAA)         # PACKET HEADER 0xAA, 0xCC
    addData('uInt8', 0xCC)
    addData('uInt8', PacketSeqNO)   # Packet Seq No.
    #--------------------------------------------------------------------
    # 보내는 센서 데이터들
    # 받는쪽에서도 아래의 순서대로 해석해야 함.
    #--------------------------------------------------------------------
    addData('uInt16', tof1V)        # TOF1 : 부호 없는 2 바이트 정수('<H').
    addData('uInt16', tof2V)        # TOF2 : 부호 없는 2 바이트 정수('<H').
    addData('uInt16', tof3V)        # TOF3 : 부호 없는 2 바이트 정수('<H').
    addData('float', imuV[0])       # YAW: 4 바이트 부동 소수('f')
    addData('float', imuV[1])       # ROLL: 4 바이트 부동 소수('f')
    addData('float', imuV[2])       # PITCH: 4 바이트 부동 소수('f')
    #--------------------------------------------------------------------
    # 보내는 이미지 데이터들
    #--------------------------------------------------------------------
    addData('uInt8', ObjectCount)   # Object Count

    if ObjectCount:                 # 보낼 Object 데이터가 있다면
        for c in SilverCircles:
            buf = [1, c.x() ,c.y() ,c.r(), c.magnitude()]   # Object Type = 1
            addData('uInt16', buf)                          # OBJECT[n]
            print(c)

        for c in BlackCircles:
            buf = [2, c.x() ,c.y() ,c.r(), c.magnitude()]   # Object Type = 2
            addData('uInt16', buf)                          # OBJECT[n]
            print(c)
    #--------------------------------------------------------------------
    # 에러 검증을 위한 체크섬 계산
    #--------------------------------------------------------------------
    chksum = 0
    for b in PACKET:
       chksum ^= b
    addData('uInt8', chksum)        #CHECKSUM
    #--------------------------------------------------------------------
    # Packet 생성 완료
    #--------------------------------------------------------------------

    # 송신하고 송신된 바이트수를 출력
    print("send",uart.write(PACKET))    # Send Packet

    # 이미지에서 찾은 OBJECT의 갯수
    print("Objects %d" % ObjectCount)

    # 송신한 PACKET을 16진수로 출력
    for p in PACKET:
        print("%2X" % p, end = " ")
    print("")

    # 프레임 처리 속도 (Frame Per Seocond)
    print("FPS %f" % clock.fps())
    print("")

#    pyb.delay(10)

    # 수신 데이터가 있다면 출력한다.
    # 이부분을 수정해서 Master로부터 명령을 받을 수 있다.
    if uart.any() > 0:
        print("RECEIVE : ",uart.read(uart.any()))
