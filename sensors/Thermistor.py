import machine


class Thermistor:
    def __init__(self, id, adc_pin):
        self.id = id
        self.adc = machine.ADC(adc_pin)
        
    def readAdc(self):
        adc_value = self.adc.read_u16()
        return adc_value
    
        
    
    