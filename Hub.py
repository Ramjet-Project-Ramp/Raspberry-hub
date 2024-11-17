

class Hub:
    def __init__(self, id, measureAllId, getMeasurentId, numberOfPressureSensors, getThermistorAdcReadingId, numberOfThermistors):
        self.id = id
        self.measureAllId = measureAllId
        self.getMeasurentId = getMeasurentId
        self.numberOfPressureSensors = numberOfPressureSensors
        self.getThermistorAdcReadingId = getThermistorAdcReadingId
        self.numberOfThermistors = numberOfThermistors
        