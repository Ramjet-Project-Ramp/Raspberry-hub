import machine

class SpiPressureSensor:
    output_measurement_command = bytes([0xAA, 0x00, 0x00])
    data_command = bytes([0xFA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    def __init__(self, id, cs_pin):
        self.id = id
        self.cs_pin = cs_pin
        self.cs = machine.Pin(self.cs_pin, machine.Pin.OUT)
        self.cs.high()
