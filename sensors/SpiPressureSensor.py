import machine

class SpiPressureSensor:
    output_measurement_command = bytes([0xAA, 0x00, 0x00])
    data_command = bytes([0xFA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    def __init__(self, id, cs_pin):
        self.id = id
        self.pin = cs_pin
        self.ss = cs = machine.Pin(self.pin, machine.Pin.OUT)
        self.ss.high()