from webthing import (Action, Event, Property, Thing, WebThingServer,SingleThing,Value)

import logging
import time
from picamera import PiCamera
from functools import partial
from flask import Flask, send_file, jsonify, request

app = Flask(__name__)
# set up the camera
camera = PiCamera()

@app.route('/properties/brightness',methods = ['GET','PUT'])
def brightness():
    if request.method == 'GET':
        payload = {'brightness': camera.brightness}
        return jsonify(payload)
    else:
        data = request.get_json()
        if 'brightness' in data and data['brightness'] in range(0,100 + 1):
            camera.brightness = data['brightness']
        return jsonify(data)

@app.route('/properties/frame',methods = ['GET'])
def shot():
    camera.capture('frame.jpg')
    return send_file('frame.jpg')

if __name__ == '__main__':
    app.run(host = '0.0.0.0')
