import util
from serial.tools import list_ports

from pydobot import Dobot

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=False)

util.goToStartPosition(device)
util.getCubeFromQueue(device)
util.goToStartPosition(device)