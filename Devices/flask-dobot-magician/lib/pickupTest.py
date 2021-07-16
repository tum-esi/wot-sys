from serial.tools import list_ports

from pydobot import Dobot
import util

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=False)

util.pickCubeUp(device)
util.goToStartPosition(device)

device.close()
