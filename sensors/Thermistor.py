import machine


class Thermistor:
    def __init__(self, id, referenceVoltage, referenceResistor, adc_pin):
        self.id = id
        self.referenceVoltage = referenceVoltage
        self.referenceResistor = referenceResistor
        self.adc = machine.ADC(adc_pin)
        
    def readResistance(self):
        adc_value = self.adc.read_u16()
        Vr = self.referenceVoltage * float(adc_value) / 65535
        resistance = self.referenceResistor * Vr / (self.referenceVoltage - Vr)
        return resistance
    
        
    
    