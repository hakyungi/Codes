#!/usr/bin/env pybricks-micropython

# 받는 PACKET의 포맷은 아래와 같음.
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

import time
import struct

from pybricks.hubs import EV3Brick
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
from pybricks.media.ev3dev import SoundFile
from tools import StopWatch, wait

PacketIndex = 0
ObjectIndex = 0

rcvPACKET = bytearray([])
Object = list([])
ObjectCount = 0

def _parsing(_data) -> tuple([uInt16, uInt16, uInt16, uInt16, uInt16]):
    buf = struct.unpack("<hhhhh", _data)
    return tuple(buf)

def readPacket() -> bool:
    global PacketIndex
    global ObjectIndex
    global rcvPACKET
    global ObjectCount
    
    read = False

    rcvPACKET = bytearray([])
    rcvPACKET += ser.read()
    PacketIndex = 0
    if rcvPACKET[0] == 0xAA:
        PacketIndex = 1
        rcvPACKET += ser.read()
        if rcvPACKET[1] == 0xCC:
            PacketIndex = 2
            rcvPACKET += ser.read()     # PACKET SeqNo

            PacketIndex = 3

            rcvPACKET += ser.read(2)    # TOF1 거리 2 Bytes Unsigned Integer
            rcvPACKET += ser.read(2)    # TOF2 거리 2 Bytes Unsigned Integer
            rcvPACKET += ser.read(2)    # TOF3 거리 2 Bytes Unsigned Integer
            rcvPACKET += ser.read(4)    # YAW 값 4 Bytes Float
            rcvPACKET += ser.read(4)    # ROLL 값 4 Bytes Float
            rcvPACKET += ser.read(4)    # PITCH 값 4 Bytes Float

            PacketIndex += 18           # 센서 데이터는 모두 18 바이트 임(위 리드한 수의 합을 더해줌).

            rcvPACKET += ser.read()     # OBJECT 갯수 1 Bytes Unsigned Integer
            ObjectCount = rcvPACKET[PacketIndex]             
            PacketIndex += 1
            ObjectIndex = PacketIndex

            timeStamp = time.time()
            while ser.waiting() < ObjectCount * 10 + 1 and time.time() - timeStamp < 100:   #100ms 이내에 모든 data가 들어오길 기다린다.
                wait(1)
            wait(1)

            if ser.waiting() <= ObjectCount * 10 + 1:     #정확한 사이즈의 DATA가 수신되지 않았음.
                print("TIME OUT")
                return False

            for i in range(ObjectCount):
                rcvPACKET += ser.read(10)

            rcvPACKET += ser.read() #CHECKSUM

            chksum = 0
            for r in rcvPACKET:     #CHECKSUM 계산
                chksum ^= r
                
            if chksum != 0:         #CHECKSUM ERROR
                print("CHECK SUM ERROR")
                return False

            return True
    return False
            
           
            
# Initialize the EV3
ev3 = EV3Brick()

# Initialize sensor port 2 as a uart device
ser = UARTDevice(Port.S3, 115200)
ser.clear()
wait(100)

while True:
    read = True
    if ser.waiting() >= 5:  # 수신 데이터가 있으면 (최소 사이즈는 5 바이트임)
        wait(1) 
        if readPacket():    # 데이터 수신에 성공했으면 Parsing 함
            tof1 = struct.unpack("<H",rcvPACKET[3:5])[0]
            tof2 = struct.unpack("<H",rcvPACKET[5:7])[0]
            tof3 = struct.unpack("<H",rcvPACKET[7:9])[0]
            yaw = struct.unpack("<f",rcvPACKET[9:13])[0]
            roll = struct.unpack("<f",rcvPACKET[13:17])[0]
            pitch = struct.unpack("<f",rcvPACKET[17:21])[0]

            object = list([])
            for n in range(ObjectCount):
                object.append(_parsing(rcvPACKET[n*10+ObjectIndex:(n+1)*10+ObjectIndex]))

            print("OK")
            print("TOF1  : ",tof1)
            print("TOF2  : ",tof2)
            print("TOF3  : ",tof3)
            print("YAW   : ",yaw)
            print("ROLL  : ",roll)
            print("PITCH : ",pitch)
            print(object)

        else:
            print("PACKET READING ERROR")
    
    ser.write("HELLO")
    wait(100)
