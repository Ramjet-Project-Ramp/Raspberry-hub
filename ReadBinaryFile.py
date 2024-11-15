import ustruct

class ReadBinaryFile:
    pressureSensorDataSize = const(3) #[Bytes]
    thermistorSensorDataSize = const(2) #[Bytes]
    lowGAccelomiterDataSize = const(6) #[Bytes]
    lowGGyroDataSize = const(6) #[Bytes]
    lowGMagnetoDataSize = const(8) #[Bytes]
    highGAccelomiterDataSize = const(6) #[Bytes]
    timeDataSize = const(4) #[Bytes]
    
    def __init__(self, numberOfPressurreSensors, numberOfThermistorSensors):
        self.numberOfPressurreSensors = numberOfPressurreSensors
        self.numberOfThermistorSensors = numberOfThermistorSensors
        
        self.pressureDataBlockSize = pressureSensorDataSize*2*numberOfPressurreSensors + timeDataSize
        self.thermisorDataBlockSize = thermistorSensorDataSize*numberOfThermistorSensors + timeDataSize
        self.acceleromiterDataSize = lowGAccelomiterDataSize+lowGGyroDataSize+lowGMagnetoDataSize+highGAccelomiterDataSize + timeDataSize
        
        self.pressureDataBlock = bytearray([224] + [0]*self.pressureDataBlockSize)
        self.thermistorDataBlock = bytearray([60] + [0]*self.thermisorDataBlockSize)
        self.accelerometerDataBlock = bytearray([7] + [0]*self.acceleromiterDataSize)
    
        
    # READ AND WRITE TO TEXT FILE    
    def readAndWriteToTextFile(self, readFileName, writeFileName):
        textFile = open(writeFileName, "w")
        binaryFile = open(readFileName, "rb")
    
        titleBlock = "Measurment_id(1),Pressure_time[ns]"

        # PRESSURE TITLE BLOCK
        for i in range(self.numberOfPressurreSensors):
            id = i+1
            titleBlock += ",pressure" + str(id) + "[counts]" + ",sensor_temperature" + str(id) + "[counts]"
            
        titleBlock += "\nMeasurment_id(2),Thermistor_time[ns]"

        for i in range(self.numberOfThermistorSensors):
            id = i+1
            titleBlock += ",temperature" + str(id) + "[counts]"
        
        # ACCELEROMETER TITLE BLOCK              
        titleBlock += "\nMeasurment_id(3),Acceleromiter_time[ns],acceleration_high_g_x[counts],acceleration_high_g_y[counts],acceleration_high_g_z[counts]"
        titleBlock += "acceleration_low_g_x[counts],acceleration_low_g_y[counts],acceleration_low_g_z[counts],gyroscope_low_g_x[counts],gyroscope_low_g_y[counts],gyroscope_low_g_z[counts],magnetometer_low_g_x[counts],magnetometer_low_g_y[counts],magnetometer_low_g_z[counts],magnetometer_r_hall[counts]"


        textFile.write(titleBlock)

        # READ BINARY FILE
        while True:
            measurment_type = binaryFile.read(1)
            if not measurment_type:
                break
            
            # PRESSURE SENSOR DATA
            if int.from_bytes(measurment_type, byteorder='big') == 224:
                textFile.write("\n" + str(224))
                
                pressureDataBlock = binaryFile.read(self.pressureDataBlockSize)
                
                time = int.from_bytes(pressureDataBlock[:timeDataSize], byteorder='big')
                textFile.write("," + str(time))
                
                for i in range(self.numberOfPressurreSensors):
                    position = i*pressureSensorDataSize*2 + timeDataSize
                    data = pressureDataBlock[position: position + pressureSensorDataSize*2]
                    press_counts = data[2] + data[1] * 256 + data[0] * 65536  # calculate digital pressure counts
                    temp_counts = data[5] + data[4] * 256 + data[3] * 65536   # calculate digital temperature counts
                    
                    textFile.write(","+str(press_counts)+","+str(temp_counts))
            
            # THERMISTOR DATA
            elif int.from_bytes(measurment_type, byteorder='big') == 60:
                textFile.write("\n" + str(60))
                
                thermistorDataBlock = binaryFile.read(self.thermisorDataBlockSize)
                
                time = int.from_bytes(thermistorDataBlock[:timeDataSize], byteorder='big')
                textFile.write("," + str(time))
                
                for i in range(self.numberOfThermistorSensors):
                    position = i*thermistorSensorDataSize + timeDataSize
                    data = thermistorDataBlock[position: position + thermistorSensorDataSize]
                    temp_counts = data[2] + data[1] * 256 #ADC READING
                    
                    textFile.write(","+str(temp_counts))
                
            # ACCELEROMETER DATA
            elif int.from_bytes(measurment_type, byteorder='big') == 7:
                textFile.write("\n" + str(7))
                
                accelerometerDataBlock = binaryFile.read(self.accelerometerDataBlock)
                
                time = int.from_bytes(accelerometerDataBlock[:timeDataSize], byteorder='big')
                textFile.write("," + str(time))
                
                start = 1 + timeDataSize 
                end = start+lowGAccelomiterDataSize
                low_G_acceleration = self.acceleromiterDataBlok[start:end]
                acc_x, acc_y, acc_z = read_accel_data(low_G_acceleration)
                
                start = end
                end += lowGGyroDataSize
                low_G_gyro = self.acceleromiterDataBlok[start:end]
                gyr_x, gyr_y, gyr_z = read_gyro_data(low_G_gyro)
                
                start = end
                end += lowGMagnetoDataSize
                low_G_magnetometer = self.acceleromiterDataBlok[start:end]
                mag_x, mag_y, mag_z, r_hall = read_mag_data(low_G_magnetometer)
                
                start = end
                end += highGAccelomiterDataSize
                high_G_acceleration = self.acceleromiterDataBlok[start:end]
                x_high_G, y_high_G, z_high_G = ustruct.unpack('<3h', high_G_acceleration)
                
                textFile.write(","+str(x_high_G)+","+str(y_high_G)+","+str(z_high_G)
                               +","+str(acc_x)+","+str(acc_y)+","+str(acc_z)
                               +","+str(gyr_x)+","+str(gyr_y)+","+str(gyr_z)
                               +","+str(mag_x)+","+str(mag_y)+","+str(mag_z)+","+str(r_hall))        

        textFile.close()
        binaryFile.close()
        
        
     # Function to read accelerometer data for all axes
    def read_accel_data(self, data:bytearray):
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
    def read_gyro_data(self, data:bytearray): # 6 bytes of gyroscope data
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
    def read_mag_data(self, data:bytearray): # 6 bytes of magnetometer
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
