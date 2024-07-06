from machine import Pin, SPI
import time

class MCP2515:
    # Constants for MCP2515 Registers and Commands
    TXB0CTRL = 0x30
    TXB0SIDH = 0x31
    TXB0SIDL = 0x32
    TXB0D0 = 0x36
    TXRTSCTRL = 0x0D
    CANCTRL = 0x0F
    CNF1 = 0x2A
    CNF2 = 0x29
    CNF3 = 0x28
    RXB0CTRL = 0x60
    RXB0SIDH = 0x61
    RXB0SIDL = 0x62
    RXB0D0 = 0x66
    CANINTF = 0x2C
    RX0IF = 0x01

    def __init__(self, spi, cs_pin, int_pin):
        self.spi = spi
        self.cs = Pin(cs_pin, Pin.OUT)
        self.int = Pin(int_pin, Pin.IN)
        self.cs.value(1)
        self.reset()
        self.configure()

    def reset(self):
        self.cs.value(0)
        self.spi.write(bytearray([0xC0]))  # Reset command
        self.cs.value(1)
        time.sleep(0.1)

    def configure(self):
        # Set CAN speed to 500 kbps (assuming 16 MHz clock)
        self.write_register(self.CANCTRL, 0x80)  # Configuration mode
        self.write_register(self.CNF1, 0x00)
        self.write_register(self.CNF2, 0x90)
        self.write_register(self.CNF3, 0x02)
        self.write_register(self.CANCTRL, 0x00)  # Normal mode

    def read_register(self, address):
        self.cs.value(0)
        self.spi.write(bytearray([0x03, address]))
        value = self.spi.read(1)[0]
        self.cs.value(1)
        return value

    def write_register(self, address, value):
        self.cs.value(0)
        self.spi.write(bytearray([0x02, address, value]))
        self.cs.value(1)

    def set_bit_modify(self, address, mask, value):
        self.cs.value(0)
        self.spi.write(bytearray([0x05, address, mask, value]))
        self.cs.value(1)

    def read_status(self):
        self.cs.value(0)
        self.spi.write(bytearray([0xA0]))
        status = self.spi.read(1)[0]
        self.cs.value(1)
        return status

    def send_message(self, id, data):
        length = len(data)
        for i in range(0, length, 8):
            self._send_single_frame(id, data[i:i+8])

    def _send_single_frame(self, id, data):
        self.write_register(self.TXB0CTRL, 0x08)  # Set TXREQ bit to 1 to request transmission
        self.write_register(self.TXB0SIDH, (id >> 3) & 0xFF)
        self.write_register(self.TXB0SIDL, (id << 5) & 0xE0)
        
        length = len(data)
        self.write_register(self.TXB0D0 - 1, length & 0x0F)  # DLC
        
        for i in range(length):
            self.write_register(self.TXB0D0 + i, data[i])
        
        # Request transmission
        self.write_register(self.TXRTSCTRL, 0x01)  # Set TXB0RTS

    def read_message(self):
        if self._check_message():
            return self._read_single_frame()
        return None

    def _check_message(self):
        status = self.read_register(self.CANINTF)
        return status & self.RX0IF

    def _read_single_frame(self):
        id_high = self.read_register(self.RXB0SIDH)
        id_low = self.read_register(self.RXB0SIDL)
        length = self.read_register(self.RXB0D0 - 1) & 0x0F
        data = []
        for i in range(length):
            data.append(self.read_register(self.RXB0D0 + i))
        
        # Clear interrupt flag
        self.set_bit_modify(self.CANINTF, self.RX0IF, 0)

        id = (id_high << 3) | (id_low >> 5)
        return {'id': id, 'data': data}

# Initialize SPI
spi = SPI(1, baudrate=1000000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(23), miso=Pin(19))

# Initialize MCP2515
mcp = MCP2515(spi, cs_pin=5, int_pin=4)

# Example: Send a CAN message with ID 0x100 and data [0x01, 0x02, 0x03, 0x04]
mcp.send_message(0x100, [0x01, 0x02, 0x03, 0x04])

# Example: Read a CAN message
message = mcp.read_message()
if message:
    print("Received message:", message)
else:
    print("No message received")
