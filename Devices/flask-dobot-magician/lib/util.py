import time
import threading

def getPosition(device):
    (x, y, z, r, j1, j2, j3, j4) = device.pose()
    (l) = device.pose_l()
    data = {"x": round(x), "y": round(y),"z": round(z),"r": round(r),"l": round(l)}
    return data

def goToStartPosition(device):
    device.move_to_with_l(150, 0, -30, -75, 450, wait=True) # we wait until this movement is done before continuing

def pushCubeQueue(device):
    device.move_to_with_l(120, -215, -30, -75, 630, wait=True) # Position above queue end to push
    device.move_to_with_l(120, -215, -90, -75, 630, wait=True) # Position behind queue end to push
    device.move_to_with_l(120, -215, -90, -75, 597, wait=True) # pushing
    device.move_to_with_l(120, -215, -90, -75, 605, wait=True) # returning a bit to not collide with cube
    device.move_to_with_l(120, -215, -30, -75, 605, wait=True) # raising arm

def pickCubeUp(device):
    device.move_to_with_l(120, -195, -30, -75, 404, wait=True) # Position above queue start
    device.suck(True)
    device.grip(False)
    device.move_to_with_l(120, -195, -85, -75, 402, wait=True) # go down to pickup cube
    device.grip(True)
    device.move_to_with_l(120, -195, -30, -75, 402, wait=True) # go up
    device.move_to_with_l(180, -50, -30, -75, 150, wait=True) # Stop between both
    device.move_to_with_l(240, 110, -30, -10, 0, wait=True) # Position above first conveyor belt
    device.move_to_with_l(240, 110, -40, -10, 0, wait=True) # put cube
    device.grip(False)
    device.move_to_with_l(240, 110, -20, -10, 0, wait=True) # go up
    device.grip(True)
    time.sleep(0.5)
    device.suck(False)

def pickAndReturnCube(device):
    device.move_to_with_l(320, -10, -20, 50, 815, wait=True) # Position above second conveyor belt
    device.suck(True)
    device.grip(False)
    device.move_to_with_l(320, -10, -40, 50, 815, wait=True) # go down to pickup
    device.grip(True)
    device.move_to_with_l(320, -10, -20, 50, 810, wait=True) # go up
    device.move_to_with_l(118, -215, -30, -75, 610, wait=True) # Postion above queue end to put
    device.move_to_with_l(118, -215, -80, -75, 610, wait=True) # put down
    device.grip(False)
    device.move_to_with_l(118, -215, -35, -75, 610, wait=True) # go up
    device.grip(True)
    time.sleep(0.5)
    device.suck(False)

def returnCubeToQueue(device):
    pickAndReturnCube(device)
    goToStartPosition(device)


def getCubeFromQueue(device):
    pickCubeUp(device)
    goToStartPosition(device)
    pushCubeQueue(device)
    goToStartPosition(device)
    # th = threading.Thread(target=thread_funtion, args=[device])
    # th.start()
    print("Getting Cube done")
    

def thread_funtion(device):
    print("Starting Pushing thread")
    goToStartPosition(device)
    pushCubeQueue(device)
    goToStartPosition(device)