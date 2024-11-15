##############################
#  _    _ _    _ ____    __  #
# | |  | | |  | |  _ \  /_ | #
# | |__| | |  | | |_) |  | | #
# |  __  | |  | |  _ <   | | #
# | |  | | |__| | |_) |  | | #
# |_|  |_|\____/|____/   |_| #
##############################                           
                           

from machine import Pin, SPI, PWM, I2C
import time
from MCP2515 import MCP2515
from Hub import Hub
from ADXL375_High_G import ADXL375_I2C
from BMX160_Low_G import BMX160
from SpiPressureSensor import SpiPressureSensor
from PressureSensorsInHub import PressureSensorsInHub
from Thermistor import Thermistor
from sdcard import SDCard
import os
import sys

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

# *****************HUBs*****************
hub2 = Hub(id=2, measureAllId=21, getMeasurentId=22, numberOfPressureSensors=4, getThermistorAdcReadingId = 23, numberOfThermistors = 1, getRaspberryThermistorAdcReadingId = 24)
hub3 = Hub(id=3, measureAllId=31, getMeasurentId=32, numberOfPressureSensors=2, getThermistorAdcReadingId = 33, numberOfThermistors = 1, getRaspberryThermistorAdcReadingId = 34)

hubs = [hub2, hub3]

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
    message = can.ReadMessage()
    
    # WAIT FOR THERMISTOR READING MESSAGE TO ARIVE
    timeSinceSendingMessage = time.time_ns()
    while message == None:
        if time.time_ns() - timeSinceSendingMessage > 0.007*1e9:
            break
        message = can.ReadMessage()
        
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
pressureSensors.append(SpiPressureSensor(id=1, cs_pin=14)) # Out Tank A1
pressureSensors.append(SpiPressureSensor(id=2, cs_pin=15)) # In Tank G1
#pSensor2 = SpiPressureSensor(id=14, cs_pin=20) # In Tank G2

pressureSensorsInHub = PressureSensorsInHub(spi_pressure_sensor,
                                            pressureSensors)

# *****************THERMISTORS*****************
thermistor1 = Thermistor(id=1, adc_pin=26)

thermistors = [thermistor1]
thermistors_length = len(thermistors)

# *****************SD CARD*****************
spi_sdcard = machine.SPI(0, baudrate=2000000, polarity=0, phase=0,
                         sck=Pin(6),
                         mosi=Pin(7),
                         miso=Pin(4))

sd = SDCard(spi_sdcard, cs=machine.Pin(5, Pin.OUT), baudrate=20000000)
os.mount(sd, '/sd')

flush_sampling_rate = 0.1 # [Hz]
lastFlushTime = time.time_ns()

i = 0
fileName = ""
while True:
    try:
        fileName = "sd/data"+str(i)+".csv"
        open(fileName, "x")
        break
    except:
        i+=1

file = open(fileName, "w")
files = os.listdir("/sd")

titleBlock = "Measurment_id(1),Pressure_time[ns]"
numberOfPressurreSensors = 27

for i in range(numberOfPressurreSensors):
    id = i+1
    titleBlock += ",pressure" + str(id) + "[counts]" + ",sensor_temperature" + str(id) + "[counts]"
    
pressureDataBlock = ["-"]*numberOfPressurreSensors
pressureSensorTemperatureDataBlock = ["-"]*numberOfPressurreSensors

numberOfThermistorSensors = 10
numberOfRaspberyThermistors = 3

titleBlock += "\nMeasurment_id(2),Thermistor_time[ns]"

for i in range(numberOfRaspberyThermistors):
    id = i+1
    titleBlock += ",Raspberry_temperature" + str(id) + "[counts]"

for i in range(numberOfThermistorSensors):
    id = i+1
    titleBlock += ",temperature" + str(id) + "[counts]"
        

thermistorDataBlock = ["-"]*(numberOfThermistorSensors+numberOfRaspberyThermistors)        
    
titleBlock += "\nMeasurment_id(3),Acceleromiter_time[ns],acceleration_high_g_x[counts],acceleration_high_g_y[counts],acceleration_high_g_z[counts]"

titleBlock += "acceleration_low_g_x[counts],acceleration_low_g_y[counts],acceleration_low_g_z[counts],gyroscope_low_g_x[counts],gyroscope_low_g_y[counts],gyroscope_low_g_z[counts],magnetometer_low_g_x[counts],magnetometer_low_g_y[counts],magnetometer_low_g_z[counts],magnetometer_r_hall[counts]"

acceleromiterDataBlok = ["-"]*13

file.write(titleBlock)
#file.flush()

# *****************MAIN LOOP*****************
last_time = time.time_ns()
while(True):
    try:
        
        writeRate = 1/((time.time_ns() - last_time)*1e-9)
        last_time = time.time_ns()
        print(writeRate)
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
                
                message = can.ReadMessage()
                # WAIT FOR CONFIRMATION MESSAGE TO ARIVE
                timeSinceSendingMessage = time.time_ns()
                while message == None:
                    if time.time_ns() - timeSinceSendingMessage > 0.007*1e9:
                        break
                    message = can.ReadMessage()

                # CHECK CONFIRMATION MESSAGE
                if message != None and message.id == 1 and message.data[0] == hub.id and message.data[1] == 1:
                    confirmedHubs.append(hub)
                    
            time.sleep(0.007) # WAIT FOR PRESSURE MEASURMENT TO BE COMPLITED
            
            # GET READINGS FROM ALL PRESSURE SENSORS IN HUB1
            for i in range(pressureSensorsInHub.numberOfSensors):
                data = pressureSensorsInHub.getMeasurement(i)
                sensor_id = data[0]
                press_counts = data[3] + data[2] * 256 + data[1] * 65536  # calculate digital pressure counts
                temp_counts = data[6] + data[5] * 256 + data[4] * 65536   # calculate digital temperature counts
                
                pressureDataBlock[sensor_id-1] = press_counts
                pressureSensorTemperatureDataBlock[sensor_id-1] = temp_counts
                
            # GET READINGS FROM ALL PRESSURE SENSORS FROM OTHER HUBS
            for hub in confirmedHubs:                
                for i in range(hub.numberOfPressureSensors):
                    can.ReadMessage() # CLEAN BUFFER
                    can.Send(hub.getMeasurentId, [i]) 
                    message = can.ReadMessage()
                    
                    # WAIT FOR PREASURE READING MESSAGE TO ARIVE
                    timeSinceSendingMessage = time.time_ns()
                    while message == None:
                        if time.time_ns() - timeSinceSendingMessage > 0.007*1e9:
                            break
                        message = can.ReadMessage()
                        
                    # CHECK PRESSURE READING    
                    if message != None and message.id == 2 and message.data[0] != 0 and len(message.data) == 7:
                        data = message.data
                        sensor_id = data[0]
                        press_counts = data[3] + data[2] * 256 + data[1] * 65536  # calculate digital pressure counts
                        temp_counts = data[6] + data[5] * 256 + data[4] * 65536   # calculate digital temperature counts
                        
                        pressureDataBlock[sensor_id-1] = press_counts
                        pressureSensorTemperatureDataBlock[sensor_id-1] = temp_counts
                        
            
            data_line = "\n1,"+str(lastPressureSensorMeasurmentTime)
            for i in range(numberOfPressurreSensors):
                data_line += ","+str(pressureDataBlock[i])+","+str(pressureSensorTemperatureDataBlock[i])
            #print(data_line)
            file.write(data_line)
        
                    
        # THERMISTOR MEASUREMNT-----------------------------------------------------------------
        if time.time_ns() - lastThermistorMeasurmentTime > (1/thermistor_sampling_rate)*1e9:
            lastThermistorMeasurmentTime = time.time_ns()
            
            # GET RASPBERRY THERMISTOR READING IN HUB1
            adc_value = machine.ADC(4).read_u16()
            #data = split_number_into_bytes(adc_value, 2)
            thermistorDataBlock[0] = adc_value
            
            # GET READINGS FROM ALL THERMISTORS IN HUB1
            for thermistor in thermistors:
                adc_value = thermistor.readAdc()
                #data = split_number_into_bytes(adc_value, 2)
                thermistorDataBlock[thermistor.id-1+numberOfRaspberyThermistors] = adc_value
            
            for hub in hubs:
                # GET RASPBERRY THERMISTOR READING
                message = getThermistorData(can, hub.getRaspberryThermistorAdcReadingId, 0)
                        
                # CHECK RASPBERRY THERMISTOR READING
                if message != None and message.id == 4 and message.data[0] != 0 and len(message.data) == 2:
                    data = message.data
                    temp = data[1] + data[0] * 256 # RASPBERRY ADC READING
                    
                    thermistorDataBlock[hub.id-1] = temp
                
                # GET READINGS FROM ALL THERMISTORS
                for i in range(hub.numberOfThermistors):
                    message = getThermistorData(can, hub.getThermistorAdcReadingId, i)
                        
                    # CHECK THERMISTOR READING
                    if message != None and message.id == 3 and message.data[0] != 0 and len(message.data) == 3:
                        data = message.data
                        sensor_id = data[0]
                        temp = data[2] + data[1] * 256 #ADC READING
                        
                        thermistorDataBlock[sensor_id-1+numberOfRaspberyThermistors] = temp
                  
            # SAVING DATA TO FILE      
            data_line = "\n2,"+str(lastThermistorMeasurmentTime)
            for data in thermistorDataBlock:
                data_line += ","+str(data)
            #print(data_line)
            file.write(data_line)
                        
                        
        # ACCELEROMETER MEASUREMNT-----------------------------------------------------------------
        if time.time_ns() - lastAccelerometerMeasurmentTime > (1/accelerometer_sampling_rate)*1e9:
            lastAccelerometerMeasurmentTime = time.time_ns()
            
            acc_high_g_x, acc_high_g_y, acc_high_g_z = adxl375.read_accel_data()
            
            acc_low_g_x, acc_low_g_y, acc_low_g_z = bmx160.read_accel_data()        
            gyr_low_g_x, gyr_low_g_y, gyr_low_g_z = bmx160.read_gyro_data()
            mag_low_g_x, mag_low_g_y, mag_low_g_z, r_hall_low_g = bmx160.read_mag_data()
            
            data_line = "\n3,"+str(lastAccelerometerMeasurmentTime)
            data_line +=  ","+str(acc_high_g_x) + ","+str(acc_high_g_y) + "," +str(acc_high_g_z)
            data_line +=  ","+str(acc_low_g_x)  + ","+str(acc_low_g_y) + "," +str(acc_low_g_z)
            data_line +=  ","+str(gyr_low_g_x)  + ","+str(gyr_low_g_y) + "," +str(gyr_low_g_z)
            data_line +=  ","+str(mag_low_g_x)  + ","+str(mag_low_g_y) + "," +str(mag_low_g_z) + "," +str(r_hall_low_g)
            #print(data_line)
            file.write(data_line)
          
          
      
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


