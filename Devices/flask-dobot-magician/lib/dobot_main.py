import requests
import socket
import time
import json
import _thread
import threading
import queue
from serial.tools import list_ports
from flask import Flask, Request, Response, abort,redirect, url_for,flash, jsonify

from pydobot import Dobot
from tdGenerator import getTD
import util
from jsonschema import Draft6Validator


# ---------------- CONFIG ----------------
TD_DIRECTORY_ADDRESS = "http://172.16.1.100:8080"
LISTENING_PORT = 8080

thQueue = queue.Queue(maxsize=3)
semaphore = threading.BoundedSemaphore()
def worker():
    while True:
        try:
            thread = thQueue.get_nowait()
            with semaphore:
                print(f'Working on {thread}')
                thread.start()
                thread.join()
                print(f'Finished {thread}')
                thQueue.task_done()
        except queue.Empty:
            continue



td = 0
app = Flask(__name__)
port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=False)


@app.route("/DobotMagician/")
def thingDescription():
    return jsonify(getTD(ip_addr))

@app.route("/DobotMagician/properties/position")
def getPosition():
    data = util.getPosition(device)
    return jsonify(data)

@app.route("/DobotMagician/actions/returnToStartPosition", methods=['Post'])
def goToStartPosition():
    if(thQueue.qsize() >= 3):
        abort(status=429)
    else:
        th = threading.Thread(target=util.goToStartPosition, args=[device])
        try:
            thQueue.put_nowait(th)
            return Response(status=200)
        except:
            abort(status=429)

@app.route("/DobotMagician/actions/getCube", methods=['Post'])
def getCube():
    if(thQueue.qsize() >= 3):
        abort(status=429)
    else:
        th = threading.Thread(target=util.getCubeFromQueue, args=[device])
        try:
            thQueue.put_nowait(th)
            return Response(status=200)
        except:
            abort(status=429)

@app.route("/DobotMagician/actions/returnCube", methods=['Post'])
def returnCube():
    if(thQueue.qsize() >= 3):
            abort(status=429)
    else:
        th = threading.Thread(target=util.returnCubeToQueue, args=[device])
        try:
            thQueue.put_nowait(th)
            return Response(status=200)
        except:
            abort(status=429)

# @app.route('/shutdown', methods=['Post'])
# def shutdown():
#     shutdownServer()

#############################################




##################################33
def submit_td(ip_addr, tdd_address):
    global td 
    td = get_td(ip_addr)
    print("Uploading TD to directory ...")
    while True:
        try:
            r = requests.post("{}/td".format(tdd_address), json=td)
            r.close()
            print("Got response: ", r.status_code)
            if 200 <= r.status_code <= 299:
                print("TD uploaded!")
                return
            else:
                print("TD could not be uploaded. Will try again in 15 Seconds...")
                time.sleep(15)
        except Exception as e:
            print(e)
            print("TD could not be uploaded. Will try again in 15 Seconds...")
            time.sleep(15)



    dur = 7
    count = 0
    while count < dur*(10):
        scrollphathd.clear()

        float_sec = (time.time() % 60) / 59.0
        seconds_progress = float_sec * 15

        if DISPLAY_BAR:
        # Step through 15 pixels to draw the seconds bar
            for y in range(15):

                current_pixel = min(seconds_progress, 1)
                scrollphathd.set_pixel(y + 1, 6, current_pixel * BRIGHTNESS)
                seconds_progress -= 1

            
                if seconds_progress <= 0:
                    break

        else:
        # Just display a simple dot
            scrollphathd.set_pixel(int(seconds_progress), 6, BRIGHTNESS)

    # Display the time (HH:MM) in a 5x5 pixel font
        scrollphathd.write_string(
            time.strftime("%H:%M"),
            x=0,               
            y=0,                  
            font=font5x5,          
            brightness=0.3  
        )

        if int(time.time()) % 2 == 0:
            scrollphathd.clear_rect(8, 0, 1, 5)

        scrollphathd.show()
        scrollphathd.flip(x=True, y=True)
        time.sleep(0.1)
        count = count + 1
    
    return "", 204
    scrollphathd.clear()
    scrollphathd.show()



# wait for Wifi to connect
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    try:
        # connect to router to ensure a successful connection
        s.connect(('172.16.1.1', 80))
        ip_addr = s.getsockname()[0] + ":" + str(LISTENING_PORT)
        print(ip_addr)
        break
    except OSError:
        time.sleep(5)

# Submit TD to directory
# _thread.start_new_thread(submit_td, (ip_addr, TD_DIRECTORY_ADDRESS))

# Run theading queue worker
threading.Thread(target=worker, daemon=True).start()
# Run app server
app.run(host='0.0.0.0', port=LISTENING_PORT)
