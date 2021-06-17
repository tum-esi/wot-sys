import requests
import socket
import time
import json
import _thread
from flask import Flask, request, abort,redirect, url_for,flash

from floraTD import get_td

from sys import exit

import board
import adafruit_tcs34725

from gpiozero import LED


# ---------------- CONFIG ----------------
TD_DIRECTORY_ADDRESS = "http://172.16.1.100:8080"
LISTENING_PORT = 8080
DEFAULT_ENABLE = 17


td = 0
app = Flask(__name__)
led = LED(DEFAULT_ENABLE)
led.on()
    
# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = adafruit_tcs34725.TCS34725(i2c,address=0x29)


@app.route("/")
def thing_description():
    return json.dumps(get_td(ip_addr)), {'Content-Type': 'application/json'}


@app.route("/properties/color", methods=["GET"])
def color():
    rgb = sensor.color_rgb_bytes
    return json.dumps(rgb), {'Content-Type': 'application/json'}


@app.route("/properties/temperature", methods=["GET"])
def temperature():
    temp = sensor.color_temperature
    return json.dumps(temp), {'Content-Type': 'application/json'}

@app.route("/properties/lux", methods=["GET"])
def lux():
    lux = sensor.lux
    return json.dumps(lux), {'Content-Type': 'application/json'}



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
_thread.start_new_thread(submit_td, (ip_addr, TD_DIRECTORY_ADDRESS))

# Run app server
app.run(host='0.0.0.0', port=8080)
