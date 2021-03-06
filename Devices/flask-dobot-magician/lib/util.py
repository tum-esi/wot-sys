import time

import RPi.GPIO as GPIO

HOME_X = 259.1
HOME_Y = 0
HOME_Z = -8.5
HOME_R = 0
HOME_L = 0

QUEUE_X_POSITION = 118
QUEUE_Y_POSITION = -210
BELT_ONE_X_POSITION = 250
BELT_ONE_Y_POSITION = 110
BELT_TWO_X_POSITION = 325
BELT_TW0_Y_POSITION = 0

#---------------Setup---------------#
GPIO.setmode(GPIO.BCM)
SWITCH_GPIO = 17
GPIO.setup(SWITCH_GPIO, GPIO.IN)
GPIO.add_event_detect(SWITCH_GPIO, GPIO.FALLING)

def getPosition(device):
    (x, y, z, r, j1, j2, j3, j4) = device.pose()
    (l) = device.pose_l()
    data = {"x": round(x), "y": round(y),"z": round(z),"r": round(r),"l": round(l)}
    return data

def calibrateDevice(device):
    device.home()
    goToStartPosition(device)

def goToStartPosition(device):
    device.move_to_with_l(HOME_X, HOME_Y, -30, -75, 450, wait=True)

def pushCubeQueue(device):
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -30, -73, 643, wait=True) # Position above queue end to push
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -90, -73, 643, wait=True) # Position behind queue end to push
    missingCubes=0
    fallingEdgeFlagTest = False
    fallingEdgeFlagPush = False
    while (not fallingEdgeFlagPush and not fallingEdgeFlagTest) and missingCubes < 10:
        lForPush = 608 - (missingCubes*25)
        device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -90, -75, lForPush, wait=True) # pushing
        channel1 = GPIO.wait_for_edge(SWITCH_GPIO, GPIO.FALLING, timeout=250)
        device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -90, -75, lForPush + 10, wait=True) # returning a bit to not collide with cube
        if channel1 is None:
            print('Timeout occurred')
        else:
            fallingEdgeFlagPush = True
            print('Edge detected on channel', channel1)
        
        if not fallingEdgeFlagPush:
            lForPush = lForPush - 6
            device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -90, -75, lForPush, wait=True) # pushing
            channel2 = GPIO.wait_for_edge(SWITCH_GPIO, GPIO.FALLING, timeout=250)
            device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -90, -75, lForPush + 10, wait=True) # returning a bit to not collide with cube
            if channel2 is None:
                print('Timeout occurred')
            else:
                fallingEdgeFlagPush = True
                print('Edge detected on channel', channel2)

        missingCubes = missingCubes + 1
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -30, -75, lForPush + 6, wait=True) # raising arm

def pickCubeUp(device):
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -30, -75, 398, wait=True) # Position above queue start
    device.suck(True)
    device.grip(False)
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -85, -75, 398, wait=True) # go down to pickup cube
    device.grip(True)
    time.sleep(0.5)
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -80, -75, 398, wait=True) # go up a little
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -80, -75, 403, wait=True) # push cubes back to not get stuck
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -80, -75, 398, wait=True) # go back to start of queue
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -30, -75, 398, wait=True) # go up
    device.move_to_with_l(HOME_X, HOME_Y, -30, -75, 150, wait=True) # Stop between both
    device.move_to_with_l(BELT_ONE_X_POSITION, BELT_ONE_Y_POSITION, -30, -10, 0, wait=True) # Position above first conveyor belt
    device.move_to_with_l(BELT_ONE_X_POSITION, BELT_ONE_Y_POSITION, -40, -10, 0, wait=True) # put cube
    device.grip(False)
    device.move_to_with_l(BELT_ONE_X_POSITION, BELT_ONE_Y_POSITION, -20, -10, 0, wait=True) # go up
    device.grip(True)
    time.sleep(0.5)
    device.suck(False)

def pickAndReturnCube(device):
    device.move_to_with_l(BELT_TWO_X_POSITION, BELT_TW0_Y_POSITION, -20, 50, 830, wait=True) # Position above second conveyor belt
    device.suck(True)
    device.grip(False)
    device.move_to_with_l(BELT_TWO_X_POSITION, BELT_TW0_Y_POSITION, -40, 50, 830, wait=True) # go down to pickup
    device.grip(True)
    device.move_to_with_l(BELT_TWO_X_POSITION, BELT_TW0_Y_POSITION, -20, 50, 830, wait=True) # go up
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -30, -75, 623.2, wait=True) # Postion above queue end to put
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -84, -75, 623.2, wait=True) # put down
    device.grip(False)
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -35, -75, 623.2, wait=True) # go up
    device.grip(True)
    time.sleep(0.5)
    device.suck(False)
    #Push
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -30, -73, 643, wait=True) # Position above queue end to push
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -90, -73, 643, wait=True) # Position behind queue end to push
    fallingEdgeFlag = False
    lForPush = 643 - 16
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -90, -75, lForPush, wait=True) # pushing
    channel = GPIO.wait_for_edge(SWITCH_GPIO, GPIO.FALLING, timeout=250)
    device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -90, -75, lForPush + 10, wait=True) # returning a bit to not collide with cube
    if channel is None:
        print('Timeout occurred')
    else:
        fallingEdgeFlag = True
        device.move_to_with_l(QUEUE_X_POSITION, QUEUE_Y_POSITION, -30, -75, lForPush + 10, wait=True) # raising arm
        print('Edge detected on channel', channel)
    return fallingEdgeFlag
    

def returnCubeToQueue(device):
    isFullyPushed = pickAndReturnCube(device)
    if not isFullyPushed:
        pushCubeQueue(device)
    goToStartPosition(device)


def getCubeFromQueue(device):
    pickCubeUp(device)
    goToStartPosition(device)
    pushCubeQueue(device)
    goToStartPosition(device)
    print("Getting Cube done")
    

def thread_funtion(device):
    print("Starting Pushing thread")
    goToStartPosition(device)
    pushCubeQueue(device)
    goToStartPosition(device)
