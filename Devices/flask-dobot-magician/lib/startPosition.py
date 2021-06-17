from serial.tools import list_ports

import util
from pydobot import Dobot

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)

util.goToStartPosition(device)
device.close()
