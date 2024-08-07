from machine import Pin, SPI, PWM, I2C
import time
from MCP2515 import MCP2515
from Hub import Hub
from ADXL375_HIGH_G import ADXL375_I2C
from BMX160_LOW_G import BMX160
from SpiPressureSensor import SpiPressureSensor
from PressureSensorsInHub import PressureSensorsInHub
from Thermistor import Thermistor
from sdcard import SDCard

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
spi = SPI(0, baudrate=1000000, polarity=0, phase=0, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
can = MCP2515(spi, cs_pin=5)

can.Init(speed="500KBPS")

# *****************HUBs*****************
hub2 = Hub(id=2, measureAllId=21, getMeasurentId=22, numberOfPressureSensors=3, getThermistorAdcReadingId = 23, numberOfThermistors = 1, getRaspberryThermistorAdcReadingId = 24)
hub3 = Hub(id=3, measureAllId=31, getMeasurentId=32, numberOfPressureSensors=3, getThermistorAdcReadingId = 33, numberOfThermistors = 1, getRaspberryThermistorAdcReadingId = 34)

hubs = [hub2]

# *****************SAMPLIG RATE*****************
pressure_sensor_sampling_rate = 5 # [Hz]
thermistor_sampling_rate = 20 # [Hz]
accelerometer_sampling_rate = 5 # [Hz]

lastPressureSensorMeasurmentTime = time.time_ns()
lastThermistorMeasurmentTime = time.time_ns()
lastAccelerometerMeasurmentTime = time.time_ns()

# *****************GET THERMISTORS DATA*****************
def getThermistorData(can, canId)
    can.ReadMessage() # CLEAN BUFFER
    # TAKE THERMISTOR MEASUREMENT 
    can.Send(canId, [])
    message = can.ReadMessage()
    
    # WAIT FOR THERMISTOR READING MESSAGE TO ARIVE
    timeSinceSendingMessage = time.time_ns()
    while message == None:
        if time.time_ns() - timeSinceSendingMessage > 0.007*1e9:
            break
        message = can.ReadMessage()
        
    return message

# *****************HIGH G ACCELEROMETER*****************
i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400000)

adxl375 = ADXL375_I2C(i2c)

# *****************LOW G ACCELEROMETER*****************
bmx160 = BMX160(i2c)
trim_data = bmx160.trim_data

# *****************PRESSURE SENSORS*****************
spi_pressure_sensor = machine.SPI(0, baudrate=800000, polarity=0, phase=0,
                  sck=Pin(18),
                  mosi=Pin(19),
                  miso=Pin(16))

paSensor = SpiPressureSensor(id=12, cs_pin=21) # Out Tank A1
pSensor1 = SpiPressureSensor(id=13, cs_pin=17) # In Tank G1
pSensor2 = SpiPressureSensor(id=14, cs_pin=20) # In Tank G2

pressureSensorsInHub = PressureSensorsInHub(spi_pressure_sensor,
                                            pressureSensors=[paSensor, pSensor1, pSensor2])

# *****************THERMISTORS*****************
thermistor1 = Thermistor(id=3, adc_pin=28)

thermistors = [thermistor1]
thermistors_length = len(thermistors)

# *****************SD CARD*****************
spi_sdcard = machine.SPI(0, baudrate=1000000, polarity=0, phase=0, sck=Pin(2), mosi=Pin(3), miso=Pin(4))

sd = sdcard.SDCard(spi_sdcard, cs=machine.Pin(5), baudrate=20000000)
os.mount(sd, '/sd')

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

titleBlock = "Pressure_time[ns]"
numberOfPressurreSensors = 27

for i in range(numberOfPressurreSensors):
    id = i+1
    titleBlock += ",pressure" + str(id) + "[counts]" + ",sensor_temperature" + str(id) + "[counts]" 



# *****************MAIN LOOP*****************
while(True):
    try:
        # PRESSURE SENSOR MEASUREMNT----------------------------------------------------------------
        if time.time_ns() - lastPressureSensorMeasurmentTime > (1/pressure_sensor_sampling_rate)*1e9:
            lastPressureSensorMeasurmentTime = time.time_ns()
            
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
                if message != None and message.id == 1 and message.data[0] == 2 and message.data[1] == 1:
                    time.sleep(0.007) # WAIT FOR PRESSURE MEASURMENT TO BE COMPLITED 
                    
                    press=""
                    # GET READINGS FROM ALL PRESSURE SENSORS
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
                        if message != None and message.id == 2 and message.data[0] != 0 len(message.data) == 7:
                            data = message.data
                            sensor_id = data[0]
                            press_counts = data[3] + data[2] * 256 + data[1] * 65536  # calculate digital pressure counts
                            temp_counts = data[6] + data[5] * 256 + data[4] * 65536   # calculate digital temperature counts
                            
                            if i == 0:
                                press_counts /= 4
                            press += " p" + str(i) + ": " + str(press_counts)
                        
                    print(press)
                    
        # THERMISTOR MEASUREMNT-----------------------------------------------------------------
        if time.time_ns() - lastThermistorMeasurmentTime > (1/thermistor_sampling_rate)*1e9:
            lastThermistorMeasurmentTime = time.time_ns()
            
            for hub in hubs:
                # GET RASPBERRY THERMISTOR READING
                message = getThermistorData(can, hub.getRaspberryThermistorAdcReadingId)
                        
                # CHECK RASPBERRY THERMISTOR READING
                if message != None and message.id == 3 and message.data[0] != 0 len(message.data) == 2:
                    data = message.data
                    temp = data[1] + data[0] * 256 # RASPBERRY ADC READING
                
                
                # GET READINGS FROM ALL THERMISTORS
                for i in range(hub.numberOfThermistors):
                    message = getThermistorData(can, hub.getThermistorAdcReadingId)
                        
                    # CHECK THERMISTOR READING
                    if message != None and message.id == 3 and message.data[0] != 0 len(message.data) == 3:
                        data = message.data
                        sensor_id = data[0]
                        temp = data[2] + data[1] * 256 #ADC READING
                        
        # ACCELEROMETER MEASUREMNT-----------------------------------------------------------------
        if time.time_ns() - lastAccelerometerMeasurmentTime > (1/accelerometer_sampling_rate)*1e9:
            lastAccelerometerMeasurmentTime = time.time_ns()
            
            acc_high_g_x, acc_high_g_y, acc_high_g_z = adxl375.read_accel_data()
            
            acc_low_g_x, acc_low_g_y, acc_low_g_z = bmx160.read_accel_data()        
            gyr_low_g_x, gyr_low_g_y, gyr_low_g_z = bmx160.read_gyro_data()
            mag_low_g_x, mag_low_g_y, mag_low_g_z, r_hall_low_g = bmx160.read_mag_data()
          
          
            
    except:
        if debug:
            print("Errror")
            led_pin(0)
            time.sleep(0.05)
            led_pin(1)
            time.sleep(0.1)
            led_pin(0)
            time.sleep(0.05)
            led_pin(1) 

