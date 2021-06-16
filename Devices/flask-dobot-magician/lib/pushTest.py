from os import wait
from serial.tools import list_ports

from pydobot import Dobot

import time

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)

(x, y, z, r, j1, j2, j3, j4) = device.pose()
(l) = device.pose_l()
device.move_to_with_l(150, 0, -30, -75, 500, wait=True)
device.move_to_with_l(120, -215, -30, -75, 622, wait=True) # Position above queue end to push
device.move_to_with_l(120, -215, -90, -75, 622, wait=True) # Position behind queue end to push
device.move_to_with_l(120, -215, -90, -75, 590, wait=True) # pushing
device.move_to_with_l(120, -215, -90, -75, 595, wait=True) # returning a bit to not collide with cube
device.move_to_with_l(120, -215, -30, -75, 595, wait=True) # raising arm
device.move_to_with_l(150, 0, -30, -75, 500, wait=True)
device.close()
