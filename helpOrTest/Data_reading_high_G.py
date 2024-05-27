from machine import Pin, I2C
import time
import ustruct

# Constants
ADXL345_ADDRESS = 0x53
ADXL345_POWER_CTL = 0x2D
ADXL345_DATA_FORMAT = 0x31
ADXL345_DATAX0 = 0x32
BW_RATE = 0x2C

# Initialize I2C
i2c = I2C(1, sda=Pin(14), scl=Pin(15), freq=400000)


# Initialize ADXL345
def init_adxl345():
    i2c.writeto_mem(ADXL345_ADDRESS, ADXL345_POWER_CTL, bytearray([0x08]))  # Set bit 3 to 1 to enable measurement mode
    i2c.writeto_mem(ADXL345_ADDRESS, BW_RATE, bytes([0x0A]))

    # i2c.writeto_mem(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, bytearray([0x08]))  # Set data format to full resolution, +/- 16g


# Read acceleration data
def read_accel_data():
    data = i2c.readfrom_mem(ADXL345_ADDRESS, ADXL345_DATAX0, 6)
    x, y, z = ustruct.unpack('<3h', data)
    return x, y, z


# Main loop
init_adxl345()
i = 1
sum = 0
while True:
    x, y, z = read_accel_data()
    scalingFactor = 400 / (2 ** 13 - 1)
    print("X: {}, Y: {}, Z: {}".format(x * scalingFactor + 0.03937988, y * scalingFactor + 0.178466,
                                       z * scalingFactor + 0.6068807))

    time.sleep(0.1)
