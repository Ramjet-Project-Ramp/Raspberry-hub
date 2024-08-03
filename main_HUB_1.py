from machine import Pin,SPI,PWM
import time
from MCP2515 import MCP2515
from Hub import Hub

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
        if time.time_ns() - lastThermistorMeasurmentTime > (1/accelerometer_sampling_rate)*1e9:
            lastAccelerometerMeasurmentTime = time.time_ns()
          
          
            
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

