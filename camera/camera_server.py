from picamera import PiCamera
from flask import Flask, send_file, jsonify, request
from td import generate_camera_thing
import socket
import requests
import json
import time
import _thread


size = (640, 480)
app = Flask(__name__)


@app.route('/properties/configuration', methods=['GET', 'PUT'])
def configuration():
    with PiCamera() as camera:
        global size
        if request.method == 'GET':
            payload = {'brightness': camera.brightness, 'size': {'width': size[0], 'height': size[1]}}
            return jsonify(payload)
        else:
            data = request.get_json()
            if 'brightness' in data and data['brightness'] in range(0, 100 + 1):
                camera.brightness = data['brightness']
            if 'size' in data and 'width' in data['size'] and 'height' in data['size']:
                size = (data['size']['width'], data['size']['height'])
            return jsonify(data)


@app.route('/properties/frame', methods=['GET'])
def shot():
    with PiCamera() as camera:
        print("size, {}".format(size))
        camera.capture('frame.jpg', resize=size)
        return send_file('frame.jpg')


def submit_td(td_json):
    print("Uploading TD to directory ...")
    while True:
        try:
            r = requests.post('http://192.168.0.100:8080/td', json=td_json)
            if 200 <= r.status_code <= 299:
                print("TD uploaded!")
                break
        except Exception as e:
            print(e)
            print("TD could not be uploaded. Will try again in 15 Seconds...")
            time.sleep(15)


# wait for Wifi to connect
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    try:
        s.connect(('192.168.0.1', 80))  # connect to router to ensure a successful connection
        ip_addr = s.getsockname()[0]
        break
    except OSError:
        time.sleep(3)

# register td in TD directory
td = generate_camera_thing(ip_addr, 8080).serialize()
td_json = json.loads(json.dumps(td))
print(json.dumps(td))
_thread.start_new_thread(submit_td, (td_json,))


app.run(host='0.0.0.0', port=8080)
