import machine

class Thermistor:
    # Data for thermistor data interpretation - it is not used in this code we are saving raw data in memory
    reference_voltage = 3.3
    reference_resistor = 8.66e3
    R0 = 10e3
    beta = 3380
    T0 = 25 + 273.15

    def __init__(self, id, ADC_Pin):
        self.id = id
        self.pin = ADC_Pin
        # ADC for thermistor themperature reading
        thermistor = machine.ADC(self.pin)

    def read(self):
        # Reading temperature value
        temperature_value = thermistor.read_u16()
        Vr = reference_voltage * float(temperature_value) / 65535
        Rt = reference_resistor * Vr / (reference_voltage - Vr)
        temp = beta / (math.log(Rt / (R0 * math.exp(-beta / T0))))
        Cel = temp - 273.15
        print("Temp: " + str(Cel) + ' degC')

        return temperature_value