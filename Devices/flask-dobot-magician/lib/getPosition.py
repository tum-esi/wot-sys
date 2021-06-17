from serial.tools import list_ports

from pydobot import Dobot

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=False)

(x, y, z, r, j1, j2, j3, j4) = device.pose()
(l) = device.pose_l()

print('{"x":',x,',"y":',y,',"z":',z,',"r":',r,',"l":',l,'}')
print(j1, j2, j3, j4)
device.close()
