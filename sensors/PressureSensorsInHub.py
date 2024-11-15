from SpiPressureSensor import SpiPressureSensor

class PressureSensorsInHub:
    def __init__(self, spi, pressureSensors):
        self.spi = spi
        self.pressureSensors = pressureSensors
        self.numberOfSensors = len(self.pressureSensors)
        
    def measureAll(self):
        for sensor in self.pressureSensors:
            sensor.cs.low()
        
        self.spi.write(SpiPressureSensor.output_measurement_command)
        
        for sensor in self.pressureSensors:
            sensor.cs.high()
            
    def getMeasurement(self, sensorInArrayPosition):
        if sensorInArrayPosition < self.numberOfSensors and sensorInArrayPosition>=0:
            sensor = self.pressureSensors[sensorInArrayPosition]
            data = bytearray(7)  # Define a bytearray to store received data
            sensor.cs.low()
            self.spi.write_readinto(SpiPressureSensor.data_command, data)
            sensor.cs.high()
        
            #press_counts = data[3] + data[2] * 256 + data[1] * 65536  # calculate digital pressure counts
            #temp_counts = data[6] + data[5] * 256 + data[4] * 65536   # calculate digital temperature counts
            
            measurement = [sensor.id, data[1], data[2], data[3], data[4], data[5], data[6]]
            
            return measurement
            
        return None
            
    
        
        