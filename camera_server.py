from webthing import (Action, Event, Property, Thing, WebThingServer,SingleThing,Value)

import logging
import time
from picamera import PiCamera
from functools import partial
from flask import Flask, send_file, jsonify, request

app = Flask(__name__)
# set up the camera
camera = PiCamera()
size = (640,480)
@app.route('/properties/configuration',methods = ['GET','PUT'])
def configuration():
    if request.method == 'GET':
        payload = {'brightness': camera.brightness, 'size': size}
        return jsonify(payload)
    else:
        global size
        data = request.get_json()
        if 'brightness' in data and data['brightness'] in range(0,100 + 1):
            camera.brightness = data['brightness']
        if 'size' in data and len(data['size']) == 2 and not any(map(lambda s: s < 0, data['size'])):
            size = tuple(data['size'])
        return jsonify(data)

@app.route('/properties/frame',methods = ['GET'])
def shot():
    print("size, {}".format(size))
    camera.capture('frame.jpg',resize = size)
    return send_file('frame.jpg')

if __name__ == '__main__':
    app.run(host = '0.0.0.0')
