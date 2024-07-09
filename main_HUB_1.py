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
hub2 = Hub(measureAllId=21, getMeasurentId=22, numberOfPressureSensors=3)

hubs = [hub2]

# *****************HUB2 INFO*****************
number_of_presure_sensors = 3
number_of_thermistors = 1

# *****************SAMPLIG RATE*****************
pressure_sensor_sampling_rate = 5 # [Hz]
thermistor_sampling_rate = 20 # [Hz]
rasberry_thermistor_sampling_rate = 20 # [Hz]

lastPressureSensorMeasurmentTime = time.time_ns()
lastThermistorMeasurmentTime = time.time_ns()

# *****************MAIN LOOP*****************
while(True):
    try:
        # PRESSURE SENSOR MEASUREMNT
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
                        
                        # WAIT FOR CONFIRMATION MESSAGE TO ARIVE
                        timeSinceSendingMessage = time.time_ns()
                        while message == None:
                            if time.time_ns() - timeSinceSendingMessage > 0.007*1e9:
                                break
                            message = can.ReadMessage()
                            
                            
                        if message != None and message.id == 2 and message.data[0] != 0 len(message.data) == 7:
                            data = message.data       
                            press_counts = data[3] + data[2] * 256 + data[1] * 65536  # calculate digital pressure counts
                            temp_counts = data[6] + data[5] * 256 + data[4] * 65536   # calculate digital temperature counts
                            
                            if i == 0:
                                press_counts /= 4
                            press += " p" + str(i) + ": " + str(press_counts)
                        
                    print(press)
                    
        # THERMISTOR MEASUREMNT
        if time.time_ns() - lastThermistorMeasurmentTime > (1/thermistor_sampling_rate)*1e9:
            lastThermistorMeasurmentTime = time.time_ns()
            
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

