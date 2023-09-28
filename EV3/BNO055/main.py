#!/usr/bin/env pybricks-micropython

#---------------------------------------------------------------------------
# BNO055 DATASHEET 89page : 4.5 Digital Interface 참조
# BNO055 DATASHEET 93page : 4.7 UART Protocol 참조
# BNO055 DATASHEET 20page : 3.3 Operation Modes 참조
# BNO055 DATASHEET  6page : 목차 어드레스 번지 참조
# BNO055 DATASHEET 98page : 5.3 Connection diagram UART 참조
#---------------------------------------------------------------------------

import time
import struct

from pybricks.hubs import EV3Brick
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
from pybricks.media.ev3dev import SoundFile
from tools import StopWatch, wait

from threading import Thread

ReadFail = True

def do_this():
    while True:
        print ("hello, world")
        wait(100)

def do_that():
    while True:
        print ("hello, mmmmmmmmm")
        wait(200)


#t1 = Thread(target = do_this)
#t2 = Thread(target = do_that)
#t1.start()
#t2.start()

def _read_register(register: int, length: int = 1) -> int:
        i = 0
        global ReadFail
        ReadFail = True

        while i < 10:
            ser.write(bytes([0xAA, 0x01, register, length]))
            w = StopWatch()
            now = w.time()
            while ser.waiting() < length + 2 and w.time() - now < 250:
                pass
            resp = ser.read(ser.waiting())
            if len(resp) >= 2 and resp[0] == 0xBB:
                ReadFail = False
                break
            #wait(10)
            i += 1

        if len(resp) < 2:                       #BNO055로부터의 응답은 최소 2바이트임, 2바이트보다 작으면 에러
            print("UART access error.")
        if resp[0] != 0xBB:                     #BNO055로부터의 정상적인 Data는 0xBB로 시작하며, 에러는 0xEE로 시작함.
            print('UART read error: ', resp[1])
        if length > 1:                          #요청한 Data의 사이즈가 1바이트 이상이면 Data 어레이 리턴, 아니면 1바이트만 리턴
            return resp[2:]
        return int(resp[2])

def _write_register(register: int, data: int) -> None:
        if not isinstance(data, bytes):
            data = bytes([data])
        print(bytes([0xAA, 0x00, register, len(data)]) + data)
        ser.write(bytes([0xAA, 0x00, register, len(data)]) + data)
        w = StopWatch()
        now = w.time()
        while ser.waiting() < 2 and w.time() - now < 700:
            print(ser.waiting())
            pass
        resp = ser.read(ser.waiting())
        if len(resp) < 2:
            print("UART access error.")
        if resp[0] != 0xEE or resp[1] != 0x01:
            print('UART write error: ', resp[1])

def _euler() -> Tuple[float, float, float]:

        resp = struct.unpack("<hhh", _read_register(0x1A, 6))
        return tuple(x / 16 for x in resp)

# Initialize the EV3
ev3 = EV3Brick()

# Initialize sensor port 2 as a uart device
ser = UARTDevice(Port.S2, baudrate=115200)

wait(700)
_write_register(0x3D, 0x00) #동작모드를 CONFIG로 변경
wait(50)
_write_register(0x3D, 0x08) #동작모드를 IMU로 변경
wait(50)

while True:
    wait(200)

    resp = _euler()
    if ReadFail == False:
        ev3.screen.print(int(resp[0]),int(resp[1]),int(resp[2]))
    else:
        ev3.screen.print("ERROR")
    
    print('Response', int(resp[0]),int(resp[1]),int(resp[2]))
    



