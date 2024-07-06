from SpiPressureSensor import SpiPressureSensor

class PressureSensorsInHub:
    def __init__(self, spi, pressureSensors):
        self.spi = spi
        self.pressureSensors = pressureSensors
        self.numberOfSensors = len(self.pressureSensors)
        
    def measureAll(self):
        for sensor in self.pressureSensors:
            sensor.ss.low()
        
        self.spi.write(SpiPressureSensor.output_measurement_command)
        
        for sensor in self.pressureSensors:
            sensor.ss.high()
            
    def getMeasurement(self, sensorInArrayPosition):
        if sensorInArrayPosition < self.numberOfSensors:
            sensor = self.pressureSensors[sensorInArrayPosition]
            data = bytearray(7)  # Define a bytearray to store received data
            sensor.ss.low()
            spi.write_readinto(SpiPressureSensor.data_command, data)
            sensor.ss.high()
        
            press_counts = data[3] + data[2] * 256 + data[1] * 65536  # calculate digital pressure counts
            temp_counts = data[6] + data[5] * 256 + data[4] * 65536   # calculate digital temperature counts
            
            return {'id': sensor.id, 'press_counts': press_counts, 'temp_counts': temp_counts}
            
        return None
            
    
        
        