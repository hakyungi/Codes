import struct
from time import sleep_ms

class TCS3472:
    def __init__(self, i2c, address=0x29):
        self.i2c = i2c
        self.address = address

#        self.i2c.start()

        sleep_ms(10)

        self.write_registers(0x00 | 0x80, b'\x03')  # RGBC Enable, Power ON
#        self.write_registers(0x01 | 0x80, b'\xFF') # SET RGBC time 2.4ms(default)
#        self.write_registers(0x01 | 0x80, b'\x2B') # I don't know why set to 511.2ms

#---------------------------------------------------------------------
# I2C의 타잎(pyb.I2C, macine.SoftI2C)에 따라서 아래 내용을 수정해 줘야 함.
#---------------------------------------------------------------------
    def read_registers(self, register, size=1):
        return self.i2c.mem_read(size, self.address, register)      #pyb,I2C
#        return self.i2c.readfrom_mem(self.address, register, size) #machine.SoftI2C

    def write_registers(self, register, data):
        self.i2c.mem_write(data, self.address, register)    #pyb.I2C
#        self.i2c.writeto_mem(self.address, register, data) #machine.SoftI2C
#---------------------------------------------------------------------

    def scaled(self):
        crgb = self.raw()
        if crgb[0] > 0:
            return tuple(float(x) / crgb[0] for x in crgb[1:])

        return (0,0,0)

    def rgb(self):
        return tuple(int(x * 255) for x in self.scaled())

    def light(self):
        return self.raw()[0]

    def brightness(self, level=65.535):
        return int((self.light() / level))

    def valid(self):
        self.i2c.writeto(self.address, b'\x93')
        return self.i2c.readfrom(self.address, 1)[0] & 1

    def raw(self):
        try:
            data = self.read_registers(0x14 | 0xA0, size = 8)
            return struct.unpack("<HHHH", data)
        except:
            return(b'\x00\x00\x00\x00\x00\x00\x00\x00') # error
