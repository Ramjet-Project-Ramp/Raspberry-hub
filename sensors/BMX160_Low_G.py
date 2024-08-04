from machine import Pin, I2C
import time
class BMX160:
    # BMX160 I2C address
    BMX160_ADDRESS = const(0x68)

    # BMX160 accelerometer registers
    REG_CMD = const(0x7E)
    CMD_ACC_NORMAL_MODE = const(0x11)
    REG_ACC_CONF = const(0x40)
    ACC_CONF_ODR_100HZ = const(0x08)
    REG_ACC_RANGE = const(0x41)
    ACC_RANGE_16G = const(0x0C)  # ±16g range
    ACC_DATA_REG = const(0x12)    # Register to read accelerometer data

    # BMX160 gyroscope registers
    REG_GYR_CONF = const(0x42)
    GYR_CONF_ODR_100HZ = const(0x08)
    REG_GYR_RANGE = const(0x43)
    GYR_RANGE_2000DPS = const(0x00)  # ±2000 degrees per second range
    GYR_DATA_REG = const(0x0C)    # Register to read gyroscope data

    # BMX160 magnetometer registers
    REG_MAG_CONF = const(0x44)
    MAG_CONF_ODR_25HZ = const(0x06)
    MAG_DATA_REG = const(0x04)    # Register to read magnetometer data

    BMX160_COMMAND_REG_ADDR = const(0x7E)
    BMX160_MAG_IF_0_ADDR = const(0x4C)
    BMX160_MAG_IF_1_ADDR = const(0x4D)
    BMX160_MAG_IF_2_ADDR = const(0x4E)
    BMX160_MAG_IF_3_ADDR = const(0x4F)
    BMX160_MAG_CONFIG_ADDR = const(0x44)
    # Auxiliary sensor Output data rate
    BMX160_MAG_ODR_RESERVED              = const(0x00)
    BMX160_MAG_ODR_0_78HZ                = const(0x01)
    BMX160_MAG_ODR_1_56HZ                = const(0x02)
    BMX160_MAG_ODR_3_12HZ                = const(0x03)
    BMX160_MAG_ODR_6_25HZ                = const(0x04)
    BMX160_MAG_ODR_12_5HZ                = const(0x05)
    BMX160_MAG_ODR_25HZ                  = const(0x06)
    BMX160_MAG_ODR_50HZ                  = const(0x07)
    BMX160_MAG_ODR_100HZ                 = const(0x08)
    BMX160_MAG_ODR_200HZ                 = const(0x09)
    BMX160_MAG_ODR_400HZ                 = const(0x0A)
    BMX160_MAG_ODR_800HZ                 = const(0x0B)
    # Mag power mode
    BMX160_MAG_SUSPEND_MODE              = const(0x18)
    BMX160_MAG_NORMAL_MODE               = const(0x19)
    BMX160_MAG_LOWPOWER_MODE             = const(0x1A)
            

    def __init__(self, i2c):
        self.i2c = i2c
        # Initialize BMX160 sensor
        self.init_bmx160_accel()
        self.init_bmx160_gyro()
        self.init_bmx160_mag()
        time.sleep_ms(5)

    # Function to write a byte to a register
    def write_byte(self, reg, data):
        self.i2c.writeto_mem(BMX160_ADDRESS, reg, bytearray([data]))

    # Function to read 16-bit signed value (little endian)
    def read_signed_16bit_le(self, reg):
        data = self.i2c.readfrom_mem(BMX160_ADDRESS, reg, 2)
        value = (data[1] << 8) | data[0]
        if value & (1 << 15):
            value -= 1 << 16
        return value

    # Function to read accelerometer data for all axes
    def read_accel_data(self):
        data = self.i2c.readfrom_mem(BMX160_ADDRESS, ACC_DATA_REG, 6)  # Read 6 bytes of accelerometer data starting from 0x12
        acc_x = (data[1] << 8) | data[0]
        acc_y = (data[3] << 8) | data[2]
        acc_z = (data[5] << 8) | data[4]
        # Convert to signed integer
        if data[1] & 0x80:
            acc_x -= 0x10000
        if data[3] & 0x80: 
            acc_y -= 0x10000
        if data[5] & 0x80: 
            acc_z -= 0x10000
        return acc_x, acc_y, acc_z

    # Function to read gyroscope data for all axes
    def read_gyro_data(self):
        data = self.i2c.readfrom_mem(BMX160_ADDRESS, GYR_DATA_REG, 6)  # Read 6 bytes of gyroscope data starting from 0x0C
        gyr_x = (data[1] << 8) | data[0]
        gyr_y = (data[3] << 8) | data[2]
        gyr_z = (data[5] << 8) | data[4]
        # Convert to signed integer
        if  data[1] & 0x80:
            gyr_x -= 0x10000
        if data[3] & 0x80:
            gyr_y -= 0x10000
        if data[5] & 0x80:
            gyr_z -= 0x10000
        return gyr_x, gyr_y, gyr_z

    # Function to read magnetometer data for all axes
    def read_mag_data(self):
        data = self.i2c.readfrom_mem(BMX160_ADDRESS, MAG_DATA_REG, 6)  # Read 6 bytes of magnetometer data starting from 0x04
        mag_x = (data[1] << 8) | data[0]
        mag_y = (data[3] << 8) | data[2]
        mag_z = (data[5] << 8) | data[4]
        # Convert to signed integer
        if data[1] & 0x80:
            mag_x -= 0x10000
        if data[3] & 0x80:
            mag_y -= 0x10000
        if data[5] & 0x80:
            mag_z -= 0x10000
        return mag_x, mag_y, mag_z

    # Initialize BMX160 accelerometer
    def init_bmx160_accel(self):
        # Set accelerometer to normal mode
        self.write_byte(REG_CMD, CMD_ACC_NORMAL_MODE)
        # Set accelerometer range
        self.write_byte(REG_ACC_RANGE, ACC_RANGE_16G)
        # Set accelerometer configuration
        self.write_byte(REG_ACC_CONF, ACC_CONF_ODR_100HZ)

    # Initialize BMX160 gyroscope
    def init_bmx160_gyro(self):

        # Set gyroscope to normal mode
        self.i2c.writeto(BMX160_ADDRESS, bytes([0x7E, 0x15]))  # Gyroscope configuration register

        # Set gyroscope full-scale range to +/- 250 degrees/sec
        self.i2c.writeto(BMX160_ADDRESS, bytes([0x43, 0x00]))  # Gyroscope range register

        # Set gyroscope bandwidth to 230Hz
        self.i2c.writeto(BMX160_ADDRESS, bytes([0x43, 0x08]))  # Gyroscope bandwidth register


    def writeBmxReg(self, address, data):
        self.i2c.writeto_mem(BMX160_ADDRESS, address, bytes([data]))


    # Initialize BMX160 magnetometer
    def init_bmx160_mag(self):
        # see pg 25 of: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMX160-DS000.pdf
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_COMMAND_REG_ADDR, BMX160_MAG_NORMAL_MODE]))
        time.sleep(0.00065) # datasheet says wait for 650microsec
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_0_ADDR, 0x80]))
        # put mag into sleep mode
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_3_ADDR, 0x01]))
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_2_ADDR, 0x4B]))
        # set x-y to regular power preset
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_3_ADDR, 0x04]))
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_2_ADDR, 0x51]))
        # set z to regular preset
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_3_ADDR, 0x0E]))
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_2_ADDR, 0x52]))
        # prepare MAG_IF[1-3] for mag_if data mode
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_3_ADDR, 0x02]))
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_2_ADDR, 0x4C]))
        i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_1_ADDR, 0x42]))
        # Set ODR to 200 Hz
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_CONFIG_ADDR, BMX160_MAG_ODR_200HZ]))
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_0_ADDR, 0x00]))
        # put in low power mode.
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_COMMAND_REG_ADDR, BMX160_MAG_NORMAL_MODE]))
        time.sleep(0.1) # takes this long to warm up (empirically)


i2c = I2C(0, scl=Pin(13), sda=Pin(12)) 
bmx160 = BMX160(i2c)


while True:
    # Read data from all sensors
    acc_x, acc_y, acc_z = bmx160.read_accel_data()
    gyr_x, gyr_y, gyr_z = bmx160.read_gyro_data()
    mag_x, mag_y, mag_z = bmx160.read_mag_data()
    
    #print("X:", acc_x / 2048.0, ", Y:", acc_y / 2048.0, ", Z:", acc_z / 2048.0)
    #print("X:", gyr_x / 16.4, ", Y:", gyr_y / 16.4, ", Z:", gyr_z / 16.4)
    print("X:", mag_x * 0.3, ", Y:", mag_y * 0.3, ", Z:", mag_z * 0.3)
    if False:
        # Print sensor data
        print("Acceleration (m/s^2):")
        print("X:", acc_x / 2048.0)  # 16g range: ±16g / (2^11)
        print("Y:", acc_y / 2048.0)
        print("Z:", acc_z / 2048.0)

        print("Gyroscope (deg/s):")
        print("X:", gyr_x / 16.4)  # 2000 dps range: ±2000 / (2^15)
        print("Y:", gyr_y / 16.4)
        print("Z:", gyr_z / 16.4)

        print("Magnetometer (uT):")
        print("X:", mag_x * 0.3)  # 0.3 uT/LSB
        print("Y:", mag_y * 0.3)
        print("Z:", mag_z * 0.3)

    time.sleep(0.1)