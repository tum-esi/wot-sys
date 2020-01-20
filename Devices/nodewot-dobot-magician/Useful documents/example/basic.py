from serial.tools import list_ports

from pydobot import Dobot

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)

(x, y, z, r, j1, j2, j3, j4) = device.pose()
(l) = device.pose_l()
print(f'x:{x} y:{y} z:{z} r:{r} l:{l} j1:{j1} j2:{j2} j3:{j3} j4:{j4}')

#device.move_to(x + 100, y, z, r, wait=False)
#device.move_to(x + 100, y + 100, z, r, wait=False)
#device.move_to(x + 100, y + 100, z + 100, r, wait=False)
device.grip(False)
#device.move_to(x, y, z, r, wait=True)  # we wait until this movement is done before continuing
#device.grip(True)
#device.move_to(x + 20, y, z, r, wait=False)
#device.move_to(x, y, z, r, wait=True)
device.move_to_with_l(228, -65, -18, -17, 0, wait=True)
#device.move_to_with_l(228, -65, -47, -16, 0, wait=True)

device.suck(False)
#device.move_to_with_l(x, y, z, r, l, wait=True)
device.close()
