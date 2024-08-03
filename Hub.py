

class Hub:
    def __init__(self, id, measureAllId, getMeasurentId, numberOfPressureSensors, getThermistorAdcReadingId, numberOfThermistors, getRaspberryThermistorAdcReadingId):
        self.id = id
        self.measureAllId = measureAllId
        self.getMeasurentId = getMeasurentId
        self.numberOfPressureSensors = numberOfPressureSensors
        self.getThermistorAdcReadingId = getThermistorAdcReadingId
        self.numberOfThermistors = numberOfThermistors
        self.getRaspberryThermistorAdcReadingId = getRaspberryThermistorAdcReadingId