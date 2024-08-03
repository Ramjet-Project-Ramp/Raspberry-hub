from machine import Pin, I2C
import ustruct

class ADXL375_I2C:
    ADXL375_ADDRESS = 0x53
    ADXL375_POWER_CTL = 0x2D
    ADXL375_DATA_FORMAT = 0x31
    ADXL375_BW_RATE = 0x2C
    ADXL375_DATAX0 = 0x32

    def __init__(self, i2c):
        self.i2c = i2c
        self.initialize

    # Initialize ADXL345
    def initialize(self):
        self.i2c.writeto_mem(ADXL375_ADDRESS, ADXL375_POWER_CTL, bytearray([0x08]))  # Set bit 3 to 1 to enable measurement mode
        self.i2c.writeto_mem(ADXL375_ADDRESS, ADXL375_BW_RATE, bytes([0x0A]))  # Set BW_RATE to 100 Hz
        # i2c.writeto_mem(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, bytearray([0x08]))  # Set data format to full resolution, +/- 16g

    # Read acceleration data
    def read_accel_data(self):
        data = self.i2c.readfrom_mem(ADXL375_ADDRESS, ADXL375_DATAX0, 6)
        x, y, z = ustruct.unpack('<3h', data)
        return x, y, z