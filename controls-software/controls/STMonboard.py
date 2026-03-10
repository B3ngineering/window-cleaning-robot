import serial
import threading

class STMonboard:
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        self.serial = serial.Serial(port, baud)
        self._stop = threading.Event()