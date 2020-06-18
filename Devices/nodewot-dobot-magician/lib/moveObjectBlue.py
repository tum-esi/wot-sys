from serial.tools import list_ports

from pydobot import Dobot

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)

(x, y, z, r, j1, j2, j3, j4) = device.pose()
(l) = device.pose_l()

device.move_to_with_l(x, y, 20, r, l, wait=True) # we wait until this movement is done before continuing
device.move_to_with_l(300,40, 20, 20, 500, wait=True)
device.move_to_with_l(300,40, -45, 20, 500, wait=True) # we wait until this movement is done before continuing
device.grip(False)
device.suck(False)
device.close()
