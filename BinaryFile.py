class BinaryFile:
    pressureSensorDataSize = const(3) #[Bytes]
    thermistorSensorDataSize = const(2) #[Bytes]
    lowGAccelomiterDataSize = const(6) #[Bytes]
    lowGGyroDataSize = const(6) #[Bytes]
    lowGMagnetoDataSize = const(8) #[Bytes]
    highGAccelomiterDataSize = const(6) #[Bytes]
    timeDataSize = const(4) #[Bytes]
    
    def __init__(self, sd = "/sd", numberOfPressurreSensors, numberOfThermistorSensors):
        self.pressureDataBlockSize = pressureSensorDataSize*2*numberOfPressurreSensors + timeDataSize
        self.thermisorDataBlockSize = thermistorSensorDataSize*numberOfThermistorSensors + timeDataSize
        self.acceleromiterDataSize = lowGAccelomiterDataSize+lowGGyroDataSize+lowGMagnetoDataSize+highGAccelomiterDataSize + timeDataSize
        
        self.pressureDataBlock = bytearray([224] + [0]*self.pressureDataBlockSize)
        self.thermistorDataBlock = bytearray([60] + [0]*self.thermisorDataBlockSize)
        self.acceleromiterDataBlok = bytearray([7] + [0]*self.acceleromiterDataSize)
        
        i = 0
        self.fileName = ""
        while True:
            try:
                self.fileName = "sd/data"+str(i)+".skiba"
                open(self.fileName, "xb")
                break
            except:
                i+=1

        self.file = open(self.fileName, "wb")
        
    def flushDisc(self):
        self.file.flush()
        
    # PRESSURE SENSOR
    def addPressureData(self, id, pressure:bytearray, temperature:bytearray):
        pressurePosition = id*2*pressureSensorDataSize-1 + timeDataSize
        self.pressureDataBlock[pressurePosition:pressurePosition+pressureSensorDataSize] = pressure
        temperaturePosition = id*2*pressureSensorDataSize + timeDataSize
        self.pressureDataBlock[temperaturePosition:temperaturePosition+pressureSensorDataSize] = temperature
        
    def savePressureData(self, time:int):
        self.pressureDataBlock[1:1+timeDataSize] = bytearray(time.to_bytes(timeDataSize, byteorder='big'))
        self.file.write(self.pressureDataBlock)
        self.pressureDataBlock[1:] = [0]*self.pressureDataBlockSize
        
    # THERMISTOR 
    def addThermistorData(self, id, temprature:bytearray):
        thermistorPosition = id*thermistorSensorDataSize-1 + timeDataSize
        self.thermistorDataBlock[thermistorPosition:thermistorPosition+thermistorSensorDataSize] = temprature
        
    def saveThermistorData(self, time:int):
        self.thermistorDataBlock[1:1+timeDataSize] = bytearray(time.to_bytes(timeDataSize, byteorder='big'))
        self.file.write(self.thermistorDataBlock)
        self.
        self.thermistorDataBlock[:] = [0]*self.thermisorDataBlockSize
        
    # ACCELEROMETER    
    def addAcceleromiterData(self, low_G_acceleration:bytearray, low_G_gyro:bytearray, low_G_magnetometer:bytearray, high_G_acceleration:bytearray):
        start = 1 + timeDataSize
        end = start+lowGAccelomiterDataSize
        self.acceleromiterDataBlok[start:end] = low_G_acceleration
        
        start = end
        end += lowGGyroDataSize
        self.acceleromiterDataBlok[start:end] = low_G_gyro
        
        start = end
        end += lowGMagnetoDataSize
        self.acceleromiterDataBlok[start:end] = low_G_magnetometer
        
        start = end
        end += highGAccelomiterDataSize
        self.acceleromiterDataBlok[start:end] = high_G_acceleration
        
    def saveAccelerometerData(self, time:int):
        self.acceleromiterDataBlok[1: 1+timeDataSize] = bytearray(time.to_bytes(timeDataSize, byteorder='big'))
        self.file.write(self.acceleromiterDataBlok)
        self.acceleromiterDataBlok[1:] = [0]*self.acceleromiterDataSize
        
   
