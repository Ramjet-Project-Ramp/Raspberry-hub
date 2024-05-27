import machine
import ubinascii
from machine import SPI, Pin
import time
from SpiPressureSensor import SpiPressureSensor
import math

# Time at rasberry startup
startTime = time.time_ns()

# Changing LED state to indicate that it started to work
led_pin = machine.Pin("LED", machine.Pin.OUT)  # GPIO pin 25 controls the onboard LED
led_pin.toggle()

# SPI for pressure sensors
spi = machine.SPI(0, baudrate=800000, polarity=0, phase=0,
                  sck=Pin(18),
                  mosi=Pin(19),
                  miso=Pin(16))


# Defining pressure sensors
pSensor1 = SpiPressureSensor(1, 17)
pSensor2 = SpiPressureSensor(2, 20)
pASensor = SpiPressureSensor(3, 21)

pressureSensors = [pSensor1, pSensor2, pASensor]

# pressure sensor data variable
data = bytearray(7)  # Define a bytearray to store received data

# Initial index for sesor read out file
dataIndex = 0
# Looking for file index which is not taken
while True:
    try:
        f = open("data"+str(dataIndex)+ ".csv", "r")
        exists = True
        dataIndex +=1
        f.close()
    except:
        break
  
file=open("data" +str(dataIndex)+ ".csv","w") # creation and opening of a CSV file in Write mode

# Creating data lebel for file
titleblock = "t"
for sensor in pressureSensors:
    titleblock += ",p" + str(sensor.id) + ",t" + str(sensor.id)
file.write(titleblock)

j = 0
i = 0

# Frequency in [Hz]
pressureSensorSampleRate = 40

dataFlushingFrequency = 200
dataWritingFrequency = 100

# initializig block of data from sensors
dataBlock = ""
# Time for sample rate
lastSensorMeasurmentTime = 0

# MAIN LOOP
while True :

    

    # Loop to control sample rate.
    while time.time_ns() - lastSensorMeasurmentTime < (1/pressureSensorSampleRate)*1e9:
        # while waiting for next sampling flushing data from RAM to Flesh
        if i>=dataFlushingFrequency:
            file.flush()
            i = 0
            led_pin.toggle()

    # Sending commad to start reading pressure to pressure sensors
    for sensor in pressureSensors:
        sensor.ss.low()
        
    spi.write(SpiPressureSensor.output_measurement_command)
    measurmentTime = time.time_ns()-startTime
    lastSensorMeasurmentTime = time.time_ns()
    
    for sensor in pressureSensors:
        sensor.ss.high()

    # Adding measument time to file
    dataBlock += "\n" + str(int(measurmentTime/1000))

    # waiting for pressure sensors to complite measuring pressure
    timebefore = time.time_ns()
    # while waiting for next sampling flushing data from RAM to Flesh
    if i>=dataFlushingFrequency:
        file.flush()
        i = 0
        led_pin.toggle()
    i+=1
    while time.time_ns() - timebefore < 7e6:
        pass

    # reading pressure sensor data - pressure and temperature
    for sensor in pressureSensors:
        sensor.ss.low()
        spi.write_readinto(SpiPressureSensor.data_command, data)
        sensor.ss.high()
    
        press_counts = data[3] + data[2] * 256 + data[1] * 65536  # calculate digital pressure counts
        temp_counts = data[6] + data[5] * 256 + data[4] * 65536   # calculate digital temperature counts
                
        dataBlock += "," + str(press_counts) + "," + str(temp_counts)

    # writing data to file
    if j >=dataWritingFrequency:
        j=0
        try:
            file.write(dataBlock)
            dataBlock = ""
        except:
            file.close()
            print("done")
            break
    j+=1
    
    
file.flush()
file.close()