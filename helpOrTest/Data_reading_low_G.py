from machine import Pin, I2C
import time

# BMX160 I2C address
BMX160_ADDR = 0x68

# BMX160 accelerometer registers
REG_CMD = 0x7E
CMD_ACC_NORMAL_MODE = 0x11
REG_ACC_CONF = 0x40
ACC_CONF_ODR_100HZ = 0x08
REG_ACC_RANGE = 0x41
ACC_RANGE_16G = 0x0C  # ±16g range
ACC_DATA_REG = 0x12    # Register to read accelerometer data

# BMX160 gyroscope registers
REG_GYR_CONF = 0x42
GYR_CONF_ODR_100HZ = 0x08
REG_GYR_RANGE = 0x43
GYR_RANGE_2000DPS = 0x00  # ±2000 degrees per second range
GYR_DATA_REG = 0x0C    # Register to read gyroscope data

# BMX160 magnetometer registers
REG_MAG_CONF = 0x44
MAG_CONF_ODR_25HZ = 0x06
MAG_DATA_REG = 0x04    # Register to read magnetometer data


BMX160_MAG_DATA_X_LSB = const(0x04)
BMX160_MAG_DATA_X_MSB = const(0x05)
BMX160_MAG_DATA_Y_LSB = const(0x06)
BMX160_MAG_DATA_Y_MSB = const(0x07)
BMX160_MAG_DATA_Z_LSB = const(0x08)
BMX160_MAG_DATA_Z_MSB = const(0x09)

BMX160_RHALL_LSB = const(0x0A)
BMX160_RHALL_MSB = const(0x0B)

# Initialize I2C
i2c = I2C(0, scl=Pin(13), sda=Pin(12))

# Function to write a byte to a register
def write_byte(reg, data):
    i2c.writeto_mem(BMX160_ADDR, reg, bytearray([data]))

# Function to read 16-bit signed value (little endian)
def read_signed_16bit_le(reg):
    data = i2c.readfrom_mem(BMX160_ADDR, reg, 2)
    value = (data[1] << 8) | data[0]
    if value & (1 << 15):
        value -= 1 << 16
    return value

# Function to read accelerometer data for all axes
def read_accel_data():
    data = i2c.readfrom_mem(BMX160_ADDR, ACC_DATA_REG, 6)  # Read 6 bytes of accelerometer data starting from 0x12
    acc_x = (data[1] << 8) | data[0]
    acc_y = (data[3] << 8) | data[2]
    acc_z = (data[5] << 8) | data[4]
    # Convert to signed integer
    if acc_x & (1 << 11):
        acc_x -= 1 << 12
    if acc_y & (1 << 11):
        acc_y -= 1 << 12
    if acc_z & (1 << 11):
        acc_z -= 1 << 12
    return acc_x, acc_y, acc_z

# Function to read gyroscope data for all axes
def read_gyro_data():
    data = i2c.readfrom_mem(BMX160_ADDR, GYR_DATA_REG, 6)  # Read 6 bytes of gyroscope data starting from 0x0C
    gyr_x = (data[1] << 8) | data[0]
    gyr_y = (data[3] << 8) | data[2]
    gyr_z = (data[5] << 8) | data[4]
    # Convert to signed integer
    if gyr_x & (1 << 15):
        gyr_x -= 1 << 16
    if gyr_y & (1 << 15):
        gyr_y -= 1 << 16
    if gyr_z & (1 << 15):
        gyr_z -= 1 << 16
    return gyr_x, gyr_y, gyr_z

# Function to read magnetometer data for all axes
def read_bmx160_mag():
    # Read raw magnetometer data
    data_x_lsb = i2c.readfrom_mem(BMX160_ADDR, BMX160_MAG_DATA_X_LSB, 1)[0]
    data_x_msb = i2c.readfrom_mem(BMX160_ADDR, BMX160_MAG_DATA_X_MSB, 1)[0]
    data_y_lsb = i2c.readfrom_mem(BMX160_ADDR, BMX160_MAG_DATA_Y_LSB, 1)[0]
    data_y_msb = i2c.readfrom_mem(BMX160_ADDR, BMX160_MAG_DATA_Y_MSB, 1)[0]
    data_z_lsb = i2c.readfrom_mem(BMX160_ADDR, BMX160_MAG_DATA_Z_LSB, 1)[0]
    data_z_msb = i2c.readfrom_mem(BMX160_ADDR, BMX160_MAG_DATA_Z_MSB, 1)[0]
    rhall_lsb = i2c.readfrom_mem(BMX160_ADDR, BMX160_RHALL_LSB, 1)[0]
    rhall_msb = i2c.readfrom_mem(BMX160_ADDR, BMX160_RHALL_MSB, 1)[0]
    
    # Combine MSB and LSB data
    data_x = (data_x_msb << 8) | data_x_lsb
    data_y = (data_y_msb << 8) | data_y_lsb
    data_z = (data_z_msb << 8) | data_z_lsb
    rhall = (rhall_msb << 8) | rhall_lsb

    # Convert raw data to signed 16-bit values
    if data_x > 32767:
        data_x -= 65536
    if data_y > 32767:
        data_y -= 65536
    if data_z > 32767:
        data_z -= 65536
    
    return data_x, data_y, data_z, rhall

def compensate_bmx160_mag(data_x, data_y, data_z, rhall):
    # Hard iron offset compensation
    data_x = data_x * 0.00142 - 0.04
    data_y = data_y * 0.00142 - 0.04
    data_z = data_z * 0.00142 - 0.04
    
    # Soft iron offset compensation
    # Compensate for positive and negative field distortion
    data_x = data_x * (1.0 + (0.0007 * 0.6))
    data_y = data_y * (1.0 + (0.0007 * 0.6))
    data_z = data_z * (1.0 + (0.0007 * 0.6))
    
    # Calculate magnetic field strength in uT
    mag_strength = ((data_x ** 2) + (data_y ** 2) + (data_z ** 2)) ** 0.5
    
    return data_x, data_y, data_z, mag_strength

# Initialize BMX160 accelerometer
def init_bmx160_accel():
    # Set accelerometer to normal mode
    write_byte(REG_CMD, CMD_ACC_NORMAL_MODE)
    # Set accelerometer range
    write_byte(REG_ACC_RANGE, ACC_RANGE_16G)
    # Set accelerometer configuration
    write_byte(REG_ACC_CONF, ACC_CONF_ODR_100HZ)

# Initialize BMX160 gyroscope
def init_bmx160_gyro():

    # Set gyroscope to normal mode
    i2c.writeto(BMX160_ADDR, bytes([0x7E, 0x15]))  # Gyroscope configuration register

    # Set gyroscope full-scale range to +/- 250 degrees/sec
    i2c.writeto(BMX160_ADDR, bytes([0x43, 0x00]))  # Gyroscope range register

    # Set gyroscope bandwidth to 230Hz
    i2c.writeto(BMX160_ADDR, bytes([0x43, 0x08]))  # Gyroscope bandwidth register



# Magnetometer configuration registers
MAGN_IF_0_ADDR = 0x4C
MAGN_IF_1_ADDR = 0x4D
MAGN_IF_2_ADDR = 0x4E
MAGN_IF_3_ADDR = 0x4F
MAGN_CONFIG_ADDR = 0x44
    

# Initialize BMX160 magnetometer
def init_bmx160_mag():
    i2c.writeto(BMX160_ADDR, bytes([0x7E, 0x19]))
    time.sleep(0.00065)
    i2c.writeto(BMX160_ADDR, bytes([0x4C, 0x80]))
    
    i2c.writeto(BMX160_ADDR, bytes([0x4F, 0x01]))
    i2c.writeto(BMX160_ADDR, bytes([0x4E, 0x4B]))
    
    i2c.writeto(BMX160_ADDR, bytes([0x4F, 0x04]))
    i2c.writeto(BMX160_ADDR, bytes([0x4E, 0x52]))
    
    i2c.writeto(BMX160_ADDR, bytes([0x4F, 0x02]))
    i2c.writeto(BMX160_ADDR, bytes([0x4E, 0x4C]))
    i2c.writeto(BMX160_ADDR, bytes([0x4D, 0x42]))
    
    i2c.writeto(BMX160_ADDR, bytes([0x44, 0x06]))
    i2c.writeto(BMX160_ADDR, bytes([0x4C, 0x00]))
    
    i2c.writeto(BMX160_ADDR, bytes([0x7E, 0x1A]))
    time.sleep(0.1)

# Initialize BMX160 sensor
init_bmx160_mag()
init_bmx160_accel()
init_bmx160_gyro()


while True:
    # Read data from all sensors
    acc_x, acc_y, acc_z = read_accel_data()
    gyr_x, gyr_y, gyr_z = read_gyro_data()
    
    data_x, data_y, data_z, rhall = read_bmx160_mag()
    data_x, data_y, data_z, mag_strength = compensate_bmx160_mag(data_x, data_y, data_z, rhall)
    
    # Print sensor data
    #print("Acceleration (m/s^2):")
    #print("X:", acc_x / 2048.0)  # 16g range: ±16g / (2^11)
    #print("Y:", acc_y / 2048.0)
   # print("Z:", acc_z / 2048.0)

   # print("Gyroscope (deg/s):")
   # print("X:", gyr_x / 16.4)  # 2000 dps range: ±2000 / (2^15)
   # print("Y:", gyr_y / 16.4)
   # print("Z:", gyr_z / 16.4)
 
    #print("Magnetometer (uT):")
    print("X:", data_x, "Y:", data_y, "Z:", data_z, "Magnitude:", mag_strength)
    #print("Y:", mag_y / 0.3)
    #print("Z:", mag_z / 0.3)

    time.sleep(0.1)
