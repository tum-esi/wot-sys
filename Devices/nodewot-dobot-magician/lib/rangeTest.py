from serial.tools import list_ports

from pydobot import Dobot

import time

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)

device.move_to_with_l(140, 30, 45, 0, 0, wait=True) # we wait until this movement is done before continuing
device.close()
