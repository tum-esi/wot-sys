from serial.tools import list_ports

from pydobot import Dobot

import time

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)

#device.move_to_with_l(215, 18, -45, 7, 10, wait=True) # infrared1 position
device.move_to_with_l(270, 20, -45, 50, 900, wait=True) # infrared2 position
#device.move_to_with_l(300,40, -45, 20, 600, wait=True) #obj4 position
#device.move_to_with_l(300,40, -45, 20, 300, wait=True) #obj1 position
#device.move_to_with_l(300,40, -45, 20, 400, wait=True) #obj2 position
#device.move_to_with_l(300,40, -45, 20, 500, wait=True) #obj3 position
device.close()
