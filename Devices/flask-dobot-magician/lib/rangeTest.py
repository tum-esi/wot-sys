from serial.tools import list_ports

from pydobot import Dobot
from util import (BELT_ONE_X_POSITION, BELT_ONE_Y_POSITION,
                  BELT_TW0_Y_POSITION, BELT_TWO_X_POSITION, QUEUE_X_POSITION,
                  QUEUE_Y_POSITION, HOME_X, HOME_Y)

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)

# device.move_to_with_l(HOME_X, HOME_Y, -30, -75, 450, wait=True) # start positon
# device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -30, -75, 400, wait=True) # Position above queue start
# device.move_to_with_l(HOME_X, HOME_Y, -30, -75, 100, wait=True) # Stop between both
# device.move_to_with_l(BELT_ONE_X_POSITION, BELT_ONE_Y_POSITION, -20, -10, 0, wait=True) # Position above first conveyor belt
# device.move_to_with_l(120, -215, -30, -75, 620, wait=True) # Position above queue end to push
# device.move_to_with_l(120, -215, -75, -75, 620, wait=True) # Position behind queue end to push
# device.move_to_with_l(325, 0, -20, 40, 825, wait=True) # Position above second conveyor belt
# device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -30, -75, 625, wait=True) # Position above queue end to put
device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -75, -75, 645, wait=True) # Position above queue end to push
device.close()
