import time

from serial.tools import list_ports

from pydobot import Dobot

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)

(x, y, z, r, j1, j2, j3, j4) = device.pose()
(l) = device.pose_l()
device.move_to_with_l(150, 0, -30, -75, 500, wait=True)
device.move_to_with_l(125, -195, -30, -75, 394, wait=True) # Position above queue start
device.suck(True)
device.grip(False)
device.move_to_with_l(122, -195, -85, -75, 394, wait=True) # go down to pickup cube
device.grip(True)
device.move_to_with_l(122, -195, -30, -75, 394, wait=True) # go up
device.move_to_with_l(150, 0, -30, -75, 100, wait=True) # Stop between both
device.move_to_with_l(240, 110, -30, -10, 0, wait=True) # Position above first conveyor belt
device.move_to_with_l(240, 110, -40, -10, 0, wait=True) # Position above first conveyor belt
device.grip(False)
device.move_to_with_l(240, 110, -20, -10, 0, wait=True) # Position above first conveyor belt
device.grip(True)
time.sleep(0.5)
device.suck(False)
device.move_to_with_l(150, 0, -30, -75, 500, wait=True)
device.close()
