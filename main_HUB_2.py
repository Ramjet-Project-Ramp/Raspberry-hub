################################
#  _    _ _    _ ____    ___   #
# | |  | | |  | |  _ \  |__ \  #
# | |__| | |  | | |_) |    ) | #
# |  __  | |  | |  _ <    / /  #
# | |  | | |__| | |_) |  / /_  #
# |_|  |_|\____/|____/  |____| #                         
################################                     
                             

from machine import Pin,SPI
import time
from MCP2515 import MCP2515
from SpiPressureSensor import SpiPressureSensor
from PressureSensorsInHub import PressureSensorsInHub
from Thermistor import Thermistor
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

# *****************PRESSURE SENSORS SET UP*****************
spi_pressure_sensor = machine.SPI(0, baudrate=800000, polarity=0, phase=0,
                  sck=Pin(2),
                  mosi=Pin(3),
                  miso=Pin(4))

pressureSensors = []
pressureSensors.append(SpiPressureSensor(id=12, cs_pin=9)) 
pressureSensors.append(SpiPressureSensor(id=13, cs_pin=8)) 
pressureSensors.append(SpiPressureSensor(id=14, cs_pin=7)) 
pressureSensors.append(SpiPressureSensor(id=15, cs_pin=6)) 
pressureSensors.append(SpiPressureSensor(id=16, cs_pin=5)) 
pressureSensors.append(SpiPressureSensor(id=17, cs_pin=1))
pressureSensors.append(SpiPressureSensor(id=18, cs_pin=0))
pressureSensors.append(SpiPressureSensor(id=30, cs_pin=22))
pressureSensors.append(SpiPressureSensor(id=31, cs_pin=21))
pressureSensors.append(SpiPressureSensor(id=32, cs_pin=20))

pressureSensorsInHub = PressureSensorsInHub(spi_pressure_sensor, pressureSensors)

# *****************THERMISTOR SENSORS SET UP*****************
thermistors = []
thermistors.append(Thermistor(id=6, adc_pin=26))
thermistors.append(Thermistor(id=7, adc_pin=27))
thermistors.append(Thermistor(id=8, adc_pin=28))
thermistors.append(Thermistor(id=9, adc_pin=4))

thermistors_length = len(thermistors)

# *****************SPLIT INTO ARRAY OF BYTES*****************
def split_number_into_bytes(number, num_bytes):
    bytes_array = [(number >> (8 * i)) & 0xFF for i in range(num_bytes)]
    return bytes_array[::-1]

# *****************MAIN LOOP*****************
while(True):
    try:
        message = can.ReadMessage()
        
        if message == None: # No message
            continue
        
        elif message.id == 21: # Take measurement from all pressure sensors
            pressureSensorsInHub.measureAll()
            can.Send(1, [2, 1])
            
        elif message.id == 22: # Get data from one pressure sensor
            measurement = pressureSensorsInHub.getMeasurement(message.data[0])
            if measurement == None:
                can.Send(2, [0])
            else:
                can.Send(2, measurement)
                
        elif message.id == 23: # Get thermistor adc reading
            if message.data[0] < thermistors_length and message.data[0] >= 0:
                thermistor = thermistors[message.data[0]]
                adc_value = thermistor.readAdc()
                data = split_number_into_bytes(adc_value, 2)
                data.insert(0, thermistor.id) # add thermistor id at the beginning
                can.Send(3,  data)
            else:
                can.Send(3, [0])
        
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




