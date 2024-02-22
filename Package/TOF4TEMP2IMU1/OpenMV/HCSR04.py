import pyb
from pyb import Pin

class HCSR04:
    def __init__(self, ch):
        if ch in ("J7" , "j7"):
            self.trig = Pin('P2', Pin.OUT_PP)            #초음파 J7
            self.echo = Pin('P3', Pin.IN, Pin.PULL_DOWN)
        elif ch in ("J8" , "j8"):
            self.trig = Pin('P4', Pin.OUT_PP)            #초음파 J8
            self.echo = Pin('P5', Pin.IN, Pin.PULL_DOWN)
        else:
            self.trig = Pin('P7', Pin.OUT_PP)            #초음파 J9
            self.echo = Pin('P8', Pin.IN, Pin.PULL_DOWN)

    def distance(self):
        pulse_start = 0
        pulse_end = 0
        pulse_dur = 0

        self.trig.value(1)
        pyb.udelay(10)
        self.trig.value(0)

        limit=20000 #측정 제한시간
        timer= pyb.micros()
        while self.echo.value() == 0 and timer+limit>pyb.micros():
            pulse_start = pyb.micros()

        timer= pyb.micros()
        while self.echo.value() == 1 and timer+limit>pyb.micros():
            pulse_end = pyb.elapsed_micros(pulse_start)

        pulse_dur = float(pulse_end)
        distance = (pulse_dur) / 58.88  #340m/s , 58.88us / 2cm(거리 1cm)
        if distance>255:                #최대 측정거리 255cm (1Byte size)
            distance=255

        while timer+limit > pyb.micros():
            pass

        return int(distance)
