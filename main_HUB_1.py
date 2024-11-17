##############################
#  _    _ _    _ ____    __  #
# | |  | | |  | |  _ \  /_ | #
# | |__| | |  | | |_) |  | | #
# |  __  | |  | |  _ <   | | #
# | |  | | |__| | |_) |  | | #
# |_|  |_|\____/|____/   |_| #
##############################                           
                           

from machine import Pin, SPI, PWM, I2C
from Hub import Hub
from MCP2515 import MCP2515
from ADXL375_High_G import ADXL375_I2C
from BMX160_Low_G import BMX160
from SpiPressureSensor import SpiPressureSensor
from PressureSensorsInHub import PressureSensorsInHub
from Thermistor import Thermistor
from sdcard import SDCard
from BinaryFile import BinaryFile
import os
import sys
import time

# *****************DEBUG MODE*****************
debug = True 


# *****************LED*****************
led_pin = machine.Pin("LED", machine.Pin.OUT)  # GPIO pin 25 controls the onboard LED

led_pin(0)
led_pin(1)
time.sleep(0.1)
led_pin(0)
time.sleep(0.05)
led_pin(1)
time.sleep(0.1)
led_pin(0)
time.sleep(1)
led_pin(1)

# *****************CAN*****************
spi = SPI(1, baudrate=1000000, polarity=0, phase=0, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
can = MCP2515(spi, cs_pin=13)

can.Init(speed="500KBPS")

def waitForCanMessage(waitMaxTimeInNs = 0.002*1e9):
    message = can.ReadMessage()
    
    timeSinceSendingMessage = time.time_ns()
    while message == None:
        if time.time_ns() - timeSinceSendingMessage > waitMaxTimeInNs:
            break
        message = can.ReadMessage()
        
    return message

# *****************HUBs*****************
hub2 = Hub(id=2, measureAllId=21, getMeasurentId=22, numberOfPressureSensors=10, getThermistorAdcReadingId = 23, numberOfThermistors = 4)
hub3 = Hub(id=3, measureAllId=31, getMeasurentId=32, numberOfPressureSensors=12, getThermistorAdcReadingId = 33, numberOfThermistors = 4)
hub4 = Hub(id=4, measureAllId=41, getMeasurentId=42, numberOfPressureSensors=7, getThermistorAdcReadingId = 43, numberOfThermistors = 4)

hubs = [hub2, hub3, hub4]

# *****************SAMPLIG RATE*****************
pressure_sensor_sampling_rate = 20 # [Hz]
thermistor_sampling_rate = 0.0020 # [Hz]
accelerometer_sampling_rate = 0.00100 # [Hz]

#startTime = const(time.time_ns())
lastPressureSensorMeasurmentTime = time.time_ns()
lastThermistorMeasurmentTime = time.time_ns()
lastAccelerometerMeasurmentTime = time.time_ns()

# *****************GET THERMISTORS DATA*****************
def getThermistorData(can, canId, sensorNumber):
    can.ReadMessage() # CLEAN BUFFER
    # TAKE THERMISTOR MEASUREMENT 
    can.Send(canId, [sensorNumber])
    
    # WAIT FOR THERMISTOR READING MESSAGE TO ARIVE
    message = waitForCanMessage()
        
    return message

# *****************HIGH G ACCELEROMETER*****************
i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)

adxl375 = ADXL375_I2C(i2c)

# *****************LOW G ACCELEROMETER*****************
i2c_low_g = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
bmx160 = BMX160(i2c_low_g)
trim_data = bmx160.trim_data

# *****************PRESSURE SENSORS*****************
spi_pressure_sensor = machine.SPI(0, baudrate=800000, polarity=0, phase=0,
                  sck=Pin(2),
                  mosi=Pin(3),
                  miso=Pin(16))
pressureSensors = []
pressureSensors.append(SpiPressureSensor(id=1, cs_pin=22)) # Out Tank A1
pressureSensors.append(SpiPressureSensor(id=2, cs_pin=21)) # In Tank G1
pressureSensors.append(SpiPressureSensor(id=3, cs_pin=20))
pressureSensors.append(SpiPressureSensor(id=4, cs_pin=19))
pressureSensors.append(SpiPressureSensor(id=10, cs_pin=18))
pressureSensors.append(SpiPressureSensor(id=11, cs_pin=15))

pressureSensorsInHub = PressureSensorsInHub(spi_pressure_sensor,
                                            pressureSensors)
pressureSensorDataSize = const(6) #[Bytes]

# *****************THERMISTORS*****************
thermistor1 = Thermistor(id=1, adc_pin=4)

thermistors = [thermistor1]
thermistors_length = len(thermistors)

thermistorSensorDataSize = const(2) #[Bytes]

# *****************SD CARD*****************
spi_sdcard = machine.SPI(0, baudrate=2000000, polarity=0, phase=0,
                         sck=Pin(6),
                         mosi=Pin(7),
                         miso=Pin(4))

sd = SDCard(spi_sdcard, cs=machine.Pin(5, Pin.OUT), baudrate=20000000)
os.mount(sd, '/sd')

flush_sampling_rate = 0.1 # [Hz]
lastFlushTime = time.time_ns()

file = BinaryFile(sd=sd ,numberOfPressurreSensors=35, numberOfThermistorSensors=13)

# *****************MAIN LOOP*****************
last_time = time.time_ns()
while(True):
    try:
        # FLUSH SD CARD-----------------------------------------------------------------
        if time.time_ns() - lastFlushTime > (1/flush_sampling_rate)*1e9:
            lastFlushTime = time.time_ns()
            
            file.flush()
            
        
        # PRESSURE SENSOR MEASUREMNT----------------------------------------------------------------
        if time.time_ns() - lastPressureSensorMeasurmentTime > (1/pressure_sensor_sampling_rate)*1e9:
            
            lastPressureSensorMeasurmentTime = time.time_ns()
            
            confirmedHubs = []
            
            # MEASURE PRESSURE IN ALL SENSORS IN HUB1
            pressureSensorsInHub.measureAll() 
            
            # MEASURE PRESSURE IN ALL SENSORS IN OTHER HUBS
            for hub in hubs:
                can.ReadMessage() # CLEAN BUFFER
                # TAKE PRESSURE MEASUREMENT FROM ALL SENSORS
                can.Send(hub.measureAllId, [])
                
                # WAIT FOR CONFIRMATION MESSAGE TO ARIVE
                message = waitForCanMessage()

                # CHECK CONFIRMATION MESSAGE
                if message != None and message.id == 1 and message.data[0] == hub.id and message.data[1] == 1:
                    confirmedHubs.append(hub)
                    
            time.sleep(0.007) # WAIT FOR PRESSURE MEASURMENT TO BE COMPLITED
            
            # GET READINGS FROM ALL PRESSURE SENSORS IN HUB1
            for i in range(pressureSensorsInHub.numberOfSensors):
                data = pressureSensorsInHub.getMeasurement(i)
                sensor_id = data[0]
                file.addPressureData(id=sensor_id, data = data[1:1+pressureSensorDataSize])
                
            # GET READINGS FROM ALL PRESSURE SENSORS FROM OTHER HUBS
            for hub in confirmedHubs:                
                for i in range(hub.numberOfPressureSensors):
                    can.ReadMessage() # CLEAN BUFFER
                    can.Send(hub.getMeasurentId, [i])
                    
                    # WAIT FOR PREASURE READING MESSAGE TO ARIVE
                    message = waitForCanMessage()
                        
                    # CHECK PRESSURE READING    
                    if message != None and message.id == 2 and message.data[0] != 0 and len(message.data) == 7:
                        data = message.data
                        sensor_id = data[0]
                        
                        file.addPressureData(id=sensor_id, data = data[1:1+pressureSensorDataSize])      
            
            file.savePressureData(time=lastPressureSensorMeasurmentTime)
        
                    
        # THERMISTOR MEASUREMNT-----------------------------------------------------------------
        if time.time_ns() - lastThermistorMeasurmentTime > (1/thermistor_sampling_rate)*1e9:
            lastThermistorMeasurmentTime = time.time_ns()
            
            # GET READINGS FROM ALL THERMISTORS IN HUB1
            for thermistor in thermistors:
                adc_value = thermistor.readAdc()
                file.addThermistorData(id=thermistor.id, temprature=adc_value.to_bytes(thermistorSensorDataSize, byteorder='big'))
            
            for hub in hubs:   
                # GET READINGS FROM ALL THERMISTORS
                for i in range(hub.numberOfThermistors):
                    message = getThermistorData(can, hub.getThermistorAdcReadingId, i)
                        
                    # CHECK THERMISTOR READING
                    if message != None and message.id == 3 and message.data[0] != 0 and len(message.data) == 3:
                        data = message.data
                        sensor_id = data[0]
                        
                        file.addThermistorData(id=sensor_id, temprature=data[1:1+thermistorSensorDataSize])
                  
            # SAVING DATA TO FILE      
            file.saveThermistorData(time=lastThermistorMeasurmentTime)
                        
                        
        # ACCELEROMETER MEASUREMNT-----------------------------------------------------------------
        if time.time_ns() - lastAccelerometerMeasurmentTime > (1/accelerometer_sampling_rate)*1e9:
            lastAccelerometerMeasurmentTime = time.time_ns()
            
            
            # READ LOW G ACCELEROMETER READING
            can.ReadMessage() # CLEAN BUFFER
            can.Send(45, [1])
            lowG_accelerometer_data = waitForCanMessage() # WAIT FOR READING MESSAGE TO ARIVE
            
            # READ LOW G GYRO READING
            can.ReadMessage() # CLEAN BUFFER
            can.Send(45, [2])
            lowG_gyro_data = waitForCanMessage() # WAIT FOR READING MESSAGE TO ARIVE
            
            # READ LOW G MAGNETO READING
            can.ReadMessage() # CLEAN BUFFER
            can.Send(45, [3])
            lowG_mag_data = waitForCanMessage() # WAIT FOR READING MESSAGE TO ARIVE
           
            # READ HIGH G ACCELEROMETER READING
            can.ReadMessage() # CLEAN BUFFER
            can.Send(45, [4])
            highG_accelerometer_data = waitForCanMessage() # WAIT FOR READING MESSAGE TO ARIVE
           
           
           file.addAcceleromiterData(low_G_acceleration=lowG_accelerometer_data,
                                     low_G_gyro=lowG_gyro_data,
                                     low_G_magnetometer=lowG_mag_data,
                                     high_G_acceleration=highG_accelerometer_data)
           
           file.saveAccelerometerData(time=lastAccelerometerMeasurmentTime)
            #acc_low_g_x, acc_low_g_y, acc_low_g_z = bmx160.read_accel_data()        
            #gyr_low_g_x, gyr_low_g_y, gyr_low_g_z = bmx160.read_gyro_data()
            #mag_low_g_x, mag_low_g_y, mag_low_g_z, r_hall_low_g = bmx160.read_mag_data()
            
            #acc_high_g_x, acc_high_g_y, acc_high_g_z = adxl375.read_accel_data()
      
    except Exception as err:
        if debug:
            print("Errror")
            sys.print_exception(err)
            led_pin(0)
            time.sleep(0.05)
            led_pin(1)
            time.sleep(0.1)
            led_pin(0)
            time.sleep(0.05)
            led_pin(1) 


