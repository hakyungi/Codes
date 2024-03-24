#---------------------------------------------------------------------
# GY-906(MLX90614) 온도센서에서 주변온도와 대상온도를 I2C를 통해 읽어들이는 프로그램.
# MLX90614의 최대 통신속도는 100 Khz.
#---------------------------------------------------------------------
# I2C 타잎(pyb.I2C, macine.SoftI2C)에 따라서 아래 메소드를 수정해 줘야 함.
#   getReg()
#---------------------------------------------------------------------

_MLX90614_IIC_ADDR   = (0x5A)
_MLX90614_TA         = (0x06)
_MLX90614_TOBJ1      = (0x07)

class MLX90614:
  def __init__(self,i2c,address=_MLX90614_IIC_ADDR):
    self.address=address
    self.i2c=i2c

  def getObjCelsius(self):
    return self.getTemp(_MLX90614_TOBJ1)    #섭씨 대상온도

  def getEnvCelsius(self):
    return self.getTemp(_MLX90614_TA)       #섭씨 주변온도

  def getTemp(self,reg):
    temp = self.getReg(reg)*0.02-273.15     #온도 변환
    return temp

#---------------------------------------------------------------------
# I2C의 타잎(pyb.I2C, macine.SoftI2C)에 따라서 아래 내용을 수정해 줘야 함.
#---------------------------------------------------------------------
  def getReg(self,register): #센서에서 DATA 읽기
#    data = self.i2c.mem_read(3, self.address, register)    #pyb.I2C
    data = self.i2c.readfrom_mem(self.address, register, 3) #machine.SoftI2C
#---------------------------------------------------------------------

    result = (data[1]<<8) | data[0]
    return result
