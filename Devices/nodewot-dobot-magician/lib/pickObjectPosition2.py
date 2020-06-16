from serial.tools import list_ports

from pydobot import Dobot

import time

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)

(x, y, z, r, j1, j2, j3, j4) = device.pose()
(l) = device.pose_l()

device.grip(False)
device.move_to_with_l(268, 23, 20, 5, 900, wait=True) # we wait until this movement is done before continuing
device.move_to_with_l(268, 23, -47, 5, 900, wait=True) # we wait until this movement is done before continuing
device.grip(True)
time.sleep(0.5)
device.suck(False)
device.close()
