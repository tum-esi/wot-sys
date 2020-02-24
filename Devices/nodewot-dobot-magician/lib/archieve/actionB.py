from serial.tools import list_ports

from pydobot import Dobot

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)

(x, y, z, r, j1, j2, j3, j4) = device.pose()
(l) = device.pose_l()

device.grip(False)
device.move_to_with_l(228, -65, 20, -17, 422, wait=True) # we wait until this movement is done before continuing
device.move_to_with_l(228, -65, -47, -17, 422, wait=True) # we wait until this movement is done before continuing
device.grip(True)
device.move_to_with_l(228, -65, -47, -17, 422, wait=True) # we wait until this movement is done before continuing
device.move_to_with_l(228, -65, 20, -17, 422, wait=True) # we wait until this movement is done before continuing
device.move_to_with_l(228, -65, 40, -17, 211, wait=True) # we wait until this movement is done before continuing
device.grip(False)
device.suck(False)
device.close()
