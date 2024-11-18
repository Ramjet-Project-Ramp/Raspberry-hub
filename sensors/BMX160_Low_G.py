from machine import Pin, I2C
import time
class BMX160:
    # BMX160 I2C address
    BMX160_ADDRESS = const(0x68)
    REG_CMD = const(0x7E)

    # BMX160 accelerometer registers
    CMD_ACC_NORMAL_MODE = const(0x11)
    REG_ACC_CONF = const(0x40)
    REG_ACC_RANGE = const(0x41)
    ACC_DATA_REG = const(0x12)    # Register to read accelerometer data
    
    ACC_CONF_ODR_0_78HZ = const(0x01)
    ACC_CONF_ODR_1_56HZ = const(0x02)
    ACC_CONF_ODR_3_12HZ = const(0x03)
    ACC_CONF_ODR_6_25HZ = const(0x04)
    ACC_CONF_ODR_12_5HZ = const(0x05)
    ACC_CONF_ODR_25HZ   = const(0x06)
    ACC_CONF_ODR_50HZ   = const(0x07)
    ACC_CONF_ODR_100HZ  = const(0x08)
    ACC_CONF_ODR_200HZ  = const(0x09)
    ACC_CONF_ODR_400HZ  = const(0x0A)
    ACC_CONF_ODR_800HZ  = const(0x0B)
    ACC_CONF_ODR_1600HZ = const(0x0C)
    
    ACC_RANGE_2G = const(0x03)  # ±16g range
    ACC_RANGE_4G = const(0x05)  # ±16g range
    ACC_RANGE_8G = const(0x08)  # ±16g range
    ACC_RANGE_16G = const(0x0C)  # ±16g range

    # BMX160 gyroscope registers
    REG_GYR_CONF = const(0x42)
    REG_GYR_RANGE = const(0x43)
    GYR_DATA_REG = const(0x0C)    # Register to read gyroscope data
    
    CMD_GYR_NORMAL_MODE = const(0x15)
    
    GYR_CONF_ODR_25HZ   = const(0x06)
    GYR_CONF_ODR_50HZ   = const(0x07)
    GYR_CONF_ODR_100HZ  = const(0x08)
    GYR_CONF_ODR_200HZ  = const(0x09)
    GYR_CONF_ODR_400HZ  = const(0x0A)
    GYR_CONF_ODR_800HZ  = const(0x0B)
    GYR_CONF_ODR_1600HZ = const(0x0C)
    GYR_CONF_ODR_3200HZ = const(0x0D)
    
    GYRO_RANGE_125DPS = const(0x04)  # ±125 degrees per second
    GYRO_RANGE_250DPS = const(0x03)  # ±250 degrees per second
    GYRO_RANGE_500DPS = const(0x02)  # ±500 degrees per second
    GYRO_RANGE_1000DPS = const(0x01) # ±1000 degrees per second
    GYRO_RANGE_2000DPS = const(0x00) # ±2000 degrees per second
    
    GYRO_SENSITIVITY_125DPS = const(262.4)  # ±125 degrees per second
    GYRO_SENSITIVITY_250DPS = const(131.2)  # ±250 degrees per second
    GYRO_SENSITIVITY_500DPS = const(65.6)  # ±500 degrees per second
    GYRO_SENSITIVITY_1000DPS = const(32.8) # ±1000 degrees per second
    GYRO_SENSITIVITY_2000DPS = const(16.4) # ±2000 degrees per second


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
        self.init_bmx160_mag()
        self.init_bmx160_accel()
        self.init_bmx160_gyro()
        time.sleep_ms(5)
        self.trim_data = self.read_trim_data()
        
    def uint8_to_int8(self, number):
        '''!
          @brief uint8_t to int8_t
          @param number    uint8_t data to be transformed
          @return number   The transformed data
        '''
        if number <= 127:
          return number
        else:
          return number - 256
        
    def read_trim_data(self):
        trim_data = {}
        
        # Read necessary registers
        trim_x1_y1 = self.read_register(0x5D, 2)
        trim_xyz_data = self.read_register(0x62, 4)
        trim_xy1_xy2  = self.read_register(0x68, 10)
        
        # Parse the trim data
        trim_data['dig_x1'] = self.uint8_to_int8(trim_x1_y1[0])
        trim_data['dig_y1'] = self.uint8_to_int8(trim_x1_y1[1])
        trim_data['dig_x2'] = self.uint8_to_int8(trim_xyz_data[2])
        trim_data['dig_y2'] = self.uint8_to_int8(trim_xyz_data[3])
        
        temp_msb = int(trim_xy1_xy2[3]) << 8
        trim_data['dig_z1'] = int(temp_msb | trim_xy1_xy2[2])
        
        temp_msb = int(trim_xy1_xy2[1] << 8)
        trim_data['dig_z2'] = int(temp_msb | trim_xy1_xy2[0])
        
        temp_msb = int(trim_xy1_xy2[7] << 8)
        trim_data['dig_z3'] = temp_msb | trim_xy1_xy2[6]
        
        temp_msb = int(trim_xyz_data[1] << 8)
        trim_data['dig_z4'] = int(temp_msb | trim_xyz_data[0])
        
        trim_data['dig_xy1'] = trim_xy1_xy2[9]
        trim_data['dig_xy2'] = self.uint8_to_int8(trim_xy1_xy2[8])
        
        temp_msb = int((trim_xy1_xy2[5] & 0x7F) << 8)
        trim_data['dig_xyz1'] = int(temp_msb | trim_xy1_xy2[4])
        
        return trim_data
    
    def read_register(self, register, length):
        return self.i2c.readfrom_mem(BMX160_ADDRESS, register, length)

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
    
    # Function to read accelerometer data for all axes
    def read_accel_data_binary(self):
        data = self.i2c.readfrom_mem(BMX160_ADDRESS, ACC_DATA_REG, 6)  # Read 6 bytes of accelerometer data starting from 0x12
        return data

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
    
    # Function to read gyroscope data for all axes
    def read_gyro_data_binary(self):
        data = self.i2c.readfrom_mem(BMX160_ADDRESS, GYR_DATA_REG, 6)  # Read 6 bytes of gyroscope data starting from 0x0C
        return data

    # Function to read magnetometer data for all axes
    def read_mag_data(self):
        data = self.i2c.readfrom_mem(BMX160_ADDRESS, MAG_DATA_REG, 8)  # Read 6 bytes of magnetometer data starting from 0x04
        mag_x = (data[1] << 8) | data[0]
        mag_y = (data[3] << 8) | data[2]
        mag_z = (data[5] << 8) | data[4]
        r_hall = (data[7] << 6) | (data[6] >> 2)
        # Convert to signed integer
        if data[1] & 0x80:
            mag_x -= 0x10000
        if data[3] & 0x80:
            mag_y -= 0x10000
        if data[5] & 0x80:
            mag_z -= 0x10000
        return mag_x, mag_y, mag_z, r_hall
    
    # Function to read magnetometer data for all axes
    def read_mag_data_binary(self):
        data = self.i2c.readfrom_mem(BMX160_ADDRESS, MAG_DATA_REG, 8)  # Read 6 bytes of magnetometer data starting from 0x04
        return data

    # Initialize BMX160 accelerometer
    def init_bmx160_accel(self):
        # Set accelerometer to normal mode
        self.write_byte(REG_CMD, CMD_ACC_NORMAL_MODE)
        # Set accelerometer range
        self.write_byte(REG_ACC_RANGE, ACC_RANGE_16G)
        # Set accelerometer configuration
        self.write_byte(REG_ACC_CONF, ACC_CONF_ODR_200HZ)

    # Initialize BMX160 gyroscope
    def init_bmx160_gyro(self):

        # Set gyroscope to normal mode
        self.i2c.writeto(BMX160_ADDRESS, bytes([REG_CMD, CMD_GYR_NORMAL_MODE]))  # Gyroscope configuration register

        # Set gyroscope full-scale range 
        self.i2c.writeto(BMX160_ADDRESS, bytes([REG_GYR_RANGE, GYRO_RANGE_1000DPS]))  # Gyroscope range register

        # Set gyroscope bandwidth
        self.i2c.writeto(BMX160_ADDRESS, bytes([REG_GYR_RANGE, GYR_CONF_ODR_200HZ]))  # Gyroscope bandwidth register


    def writeBmxReg(self, address, data):
        self.i2c.writeto_mem(BMX160_ADDRESS, address, bytes([data]))


    # Initialize BMX160 magnetometer
    def init_bmx160_mag(self):
        # Reset the BMX160 to ensure it starts from a known state
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_COMMAND_REG_ADDR, 0xB6]))
        time.sleep(0.1)  # Wait for reset to complete
        
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
        # Set ODR to 100 Hz
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_CONFIG_ADDR, BMX160_MAG_ODR_100HZ]))
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_0_ADDR, 0x00]))
        # Finalize configuration
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_MAG_IF_0_ADDR, 0x00]))
        # put in low power mode.
        self.i2c.writeto(BMX160_ADDRESS, bytes([BMX160_COMMAND_REG_ADDR, BMX160_MAG_NORMAL_MODE]))
        time.sleep(0.1) # takes this long to warm up (empirically)
   