from os import wait
from serial.tools import list_ports

from pydobot import Dobot

import time

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)

(x, y, z, r, j1, j2, j3, j4) = device.pose()
(l) = device.pose_l()

# device.move_to_with_l(150, 0, -30, -75, 500, wait=True) # start positon
# device.move_to_with_l(125, -195, -30, -75, 394, wait=True) # Position above queue start
# device.move_to_with_l(125, -195, -75, -75, 394, wait=True) # Directly above first cube
# device.move_to_with_l(150, 0, -30, -75, 100, wait=True) # Stop between both
# device.move_to_with_l(240, 110, -20, -10, 0, wait=True) # Position above first conveyor belt
# device.move_to_with_l(120, -215, -30, -75, 620, wait=True) # Position above queue end to push
# device.move_to_with_l(120, -215, -75, -75, 620, wait=True) # Position behind queue end to push
# device.move_to_with_l(320, -10, -20, 50, 800, wait=True) # Position above second conveyor belt
device.move_to_with_l(125, -215, -30, -75, 600, wait=True) # Position above queue end to put
# device.move_to_with_l(120, -215, -75, -75, 600, wait=True) # Position above queue end to put
device.close()
