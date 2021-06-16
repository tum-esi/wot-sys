from serial.tools import list_ports
import util
from pydobot import Dobot

import time

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=False)
util.returnCubeToQueue(device)
device.close()


