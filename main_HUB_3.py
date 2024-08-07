from machine import Pin,SPI
import time
from MCP2515 import MCP2515
from SpiPressureSensor import SpiPressureSensor
from PressureSensorsInHub import PressureSensorsInHub
from Thermistor import Thermistor

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
spi_can = SPI(0,baudrate=1000000,polarity=0, phase=0,sck=Pin(6),mosi=Pin(7),miso=Pin(4))
can = MCP2515(spi_can, cs_pin = 8)

can.Init(speed="500KBPS")

# *****************PRESSURE SENSORS SET UP*****************
spi_pressure_sensor = machine.SPI(0, baudrate=800000, polarity=0, phase=0,
                  sck=Pin(18),
                  mosi=Pin(19),
                  miso=Pin(16))

paSensor = SpiPressureSensor(id=12, cs_pin=21) # Out Tank A1
pSensor1 = SpiPressureSensor(id=13, cs_pin=17) # In Tank G1
pSensor2 = SpiPressureSensor(id=14, cs_pin=20) # In Tank G2

pressureSensorsInHub = PressureSensorsInHub(spi_pressure_sensor, pressureSensors=[paSensor, pSensor1, pSensor2])

# *****************THERMISTOR SENSORS SET UP*****************
thermistor1 = Thermistor(id=3, adc_pin=28)
thermistors = [thermistor1]
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
        
        elif message.id == 31: # Take measurement from all pressure sensors
            pressureSensorsInHub.measureAll()
            can.Send(1, [3, 1])
            
        elif message.id == 32: # Get data from one pressure sensor
            measurement = pressureSensorsInHub.getMeasurement(message.data[0])
            if measurement == None:
                can.Send(2, [0])
            else:
                can.Send(2, measurement)
                
        elif message.id == 33: # Get thermistor adc reading
            if message.data[0] < thermistors_length and message.data[0] >= 0:
                thermistor = thermistors[message.data[0]]
                adc_value = thermistor.readAdc
                data = split_number_into_bytes(adc_value, 2)
                data.insert(0, thermistor.id) # add thermistor id at the beginning
                can.Send(3,  data)
            else:
                can.Send(3, [0])
            
        elif message.id == 34: # Get Raspberry Pi Pico built-in thermistor adc reading
            adc_value = machine.ADC(4).read_u16()
            data = split_number_into_bytes(adc_value, 2)
            can.Send(4,  data)
        
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



