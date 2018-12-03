import logging
import time
from picamera import PiCamera
from functools import partial
from flask import Flask, send_file, jsonify, request
from td import generate_camera_thing
import socket
import requests
import json
app = Flask(__name__)
# set up the camera
camera = PiCamera()
size = (640,480)
@app.route('/properties/configuration',methods = ['GET','PUT'])
def configuration():
    if request.method == 'GET':
        payload = {'brightness': camera.brightness, 'size': {'width': size[0], 'height': size[1]}}
        return jsonify(payload)
    else:
        global size
        data = request.get_json()
        if 'brightness' in data and data['brightness'] in range(0,100 + 1):
            camera.brightness = data['brightness']
        if 'size' in data and 'width' in data['size'] and 'height' in data['size']:
            size = (data['size']['width'], data['size']['height'])
        return jsonify(data)

@app.route('/properties/frame',methods = ['GET'])
def shot():
    print("size, {}".format(size))
    camera.capture('frame.jpg',resize = size)
    return send_file('frame.jpg')


# regiter td automatically
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(('192.168.0.1',80)) # connect to router to ensure a successful connection
ip_addr = s.getsockname()[0]

td = generate_camera_thing(ip_addr).serialize()
td_json = json.loads(json.dumps(td))
print(json.dumps(td))
while True:
    try:
        r = requests.post('http://192.168.0.100:8080/td',json = td_json)
        if r.status_code == 201:
            print("TD uploaded!")
            break
    except Exception as e:
        print(e)
        continue

app.run(host = '0.0.0.0')
