import requests
import socket
import time
import json
import _thread
from random import randint
from flask import Flask, request, abort,redirect, url_for,flash

from phatTD import get_td
import scrollphathd
from jsonschema import Draft6Validator
from sys import exit
from scrollphathd.fonts import font5x5

try:
    from PIL import Image
except ImportError:
    exit("This script requires the pillow module\nInstall with: sudo pip install pillow")

# ---------------- CONFIG ----------------
TD_DIRECTORY_ADDRESS = "http://172.16.1.100:8080"
LISTENING_PORT = 8080
DEFAULT_BRIGHTNESS = 0.5
IMAGE_BRIGHTNESS = 0.5
DISPLAY_BAR = False
BRIGHTNESS = 0.3

td = 0
app = Flask(__name__)


@app.route("/")
def thing_description():
    return json.dumps(get_td(ip_addr)), {'Content-Type': 'application/json'}


@app.route("/properties/displaySize", methods=["GET"])
def display_size():
    return json.dumps(scrollphathd.get_shape()), {'Content-Type': 'application/json'}


@app.route("/actions/setPixel", methods=["POST"])
def setPixel():
    if request.is_json:
        schema = td["actions"]["setPixel"]["input"]
        valid_input = Draft6Validator(schema).is_valid(request.json)

        if valid_input:
            x = 5
            y = 5
            
            try:
                x = int(request.json["x"])
            except Exception as e:
                    print(e)
            try:
                y = int(request.json["y"])
            except Exception as e:
                    print(e)
            try:
                bright = float(request.json["brightness"])
                scrollphathd.clear()
                scrollphathd.show()
                scrollphathd.set_pixel(x, y, bright)
                scrollphathd.show()
                return "", 204
            except Exception as e:
                print(e)
                abort(400)
        else:
            abort(400)
    else:
        abort(415)  # Wrong media type.


@app.route("/actions/writeString", methods=["POST"])
def writeString():
    if request.is_json:
        schema = td["actions"]["writeString"]["input"]
        valid_input = Draft6Validator(schema).is_valid(request.json)

        if valid_input:
            dur = 5
            count = 0
            try:
                try:
                    dur = int(request.json["time"])
                except Exception as e:
                    print(e)
                Str = " " + str(request.json["string"])
                print(Str)
                x = int(request.json["x"])
                print(x)
                y = int(request.json["y"])
                print(y)
                bright = float(request.json["brightness"])
            
                scrollphathd.clear()
                scrollphathd.show()
                scrollphathd.write_string(Str, x, y, font=None, letter_spacing=1, brightness=bright, monospaced=True, fill_background=False)
                scrollphathd.flip(x=True, y=True)

                while count < dur*10:
                    scrollphathd.show()
                    scrollphathd.scroll(1)
                    time.sleep(0.05)
                    count = count + 1
                scrollphathd.clear()
                scrollphathd.show()
                return "", 204
            except Exception as e:
                print(e)
                abort(400)
        else:
            abort(400)
            print("wrong input")
    else:
        abort(415)  # Wrong media type.


@app.route("/actions/writeChar", methods=["POST"])
def writeChar():
    if request.is_json:
        schema = td["actions"]["writeChar"]["input"]
        valid_input = Draft6Validator(schema).is_valid(request.json)

        if valid_input:
            o_x = 5
            o_y = 0
            Char = str(request.json["char"])
            try:
                o_x = int(request.json["o_x"])
            except Exception as e:
                    print(e)
            try:
                o_y = int(request.json["o_y"])
            except Exception as e:
                    print(e)
            bright = float(request.json["brightness"])

            scrollphathd.clear()
            scrollphathd.show()
            scrollphathd.draw_char(o_x, o_y, Char, font=None,brightness=bright, monospaced=True)
            scrollphathd.flip(x=True, y=True)
            scrollphathd.show()
            time.sleep(5)
            scrollphathd.clear()
            scrollphathd.show()
            return "", 204
        else:
            abort(400)
            print("wrong input")
    else:
        abort(415)  # Wrong media type.


@app.route("/actions/fill", methods=["POST"])
def fillArea():
    if request.is_json:
        schema = td["actions"]["fill"]["input"]
        valid_input = Draft6Validator(schema).is_valid(request.json)

        if valid_input:
            x = 0
            y = 0
            w = 17
            h = 7
            try:
                x = int(request.json["x"])
            except Exception as e:
                print(e)
            try:
                y = int(request.json["y"])
            except Exception as e:
                print(e)
            try:
                w = request.json["width"]
            except Exception as e:
                print(e)
            bright = float(request.json["brightness"])
            try:
                h = request.json["height"]
            except Exception as e:
                print(e)
            scrollphathd.clear()
            scrollphathd.show()
            scrollphathd.fill(brightness=bright, x=x, y=y, width=w, height=h)
            scrollphathd.show()
            return "", 204
        else:
            abort(400)
            print("wrong input")
    else:
        abort(415)  # Wrong media type.


@app.route("/actions/clearRect", methods=["POST"])
def clearArea():
    if request.is_json:
        schema = td["actions"]["clearRect"]["input"]
        valid_input = Draft6Validator(schema).is_valid(request.json)

        if valid_input:
            x = 0
            y = 0
            w = 17
            h = 6
            try:
                x = int(request.json["x"])
            except Exception as e:
                print(e)
            try:
                y = int(request.json["y"])
            except Exception as e:
                print(e)
            try:
                w = request.json["width"]
            except Exception as e:
                print(e)
            try:
                h = request.json["height"]
            except Exception as e:
                print(e)
            scrollphathd.clear_rect(x, y, w, h)
            scrollphathd.show()
            return "", 204
        else:
            abort(400)
            print("wrong input")
    else:
        abort(415)  # Wrong media type.


@app.route("/actions/clear", methods=["POST"])
def Clear():
    if request.is_json:

        scrollphathd.clear()
        scrollphathd.show()

        return "", 204
    else:
        abort(415)  # Wrong media type.


@app.route("/actions/scroll", methods=["POST"])
def scroll():
    if request.is_json:
        x = 1
        y = 0
        try:
            x = int(request.json["x"])
        except Exception as e:
            print(e)
        try:
            y = int(request.json["y"])
        except Exception as e:
            print(e)
        scrollphathd.scroll(x,y)
        scrollphathd.show()

        return "", 204
    else:
        abort(415)  # Wrong media type.

@app.route("/actions/showPulse", methods=["POST"])
def pulse():
    scrollphathd.clear()
    scrollphathd.show()
    dur = 10
    count = 0
    img = Image.open("mouth.bmp")
    img.show()
    scrollphathd.clear()
    scrollphathd.show()
    def get_pixel(x, y):
        p = img.getpixel((x, y))

        if img.getpalette() is not None:
            r, g, b = img.getpalette()[p:p + 3]
            p = max(r, g, b)

        return p / 255.0

    try:
        for x in range(0, scrollphathd.DISPLAY_WIDTH):
            for y in range(0, scrollphathd.DISPLAY_HEIGHT):
                brightness = get_pixel(x, y)
                scrollphathd.pixel(x, 6 - y, brightness * IMAGE_BRIGHTNESS)

        while count < dur*10:
            scrollphathd.show()
            scrollphathd.scroll(1)
            time.sleep(0.05)
            count = count + 1
        scrollphathd.clear()
        scrollphathd.show()
        return "", 204

    except KeyboardInterrupt:
        scrollphathd.clear()
        scrollphathd.show()

@app.route("/actions/clock", methods=["POST"])
def clock():
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
    ###########################################

@app.route("/actions/sendImage", methods=["POST"])
def send_image():
    if request.method == 'POST':
        try:
            print("headers")
            print(request.headers)
            print("data")
            print(request.data)
            print("headers")
            print(request.files)
            # uploaded_file = request.files['image']
            def get_pixel(x, y):
                p = img.getpixel((x, y))
                if img.getpalette() is not None:
                    r, g, b = img.getpalette()[p:p + 3]
                    p = max(r, g, b)
                return p / 255.0

            for x in request.files:
                print(x)
                uploaded_file = request.files[x]
                uploaded_file.save(x)
                print(uploaded_file.filename)
                img = Image.open(uploaded_file.filename)
                try:
                    for x in range(0, 17):
                        for y in range(0, 7):
                            brightness = get_pixel(x, y)
                            scrollphathd.pixel(x, 6 - y, brightness * 0.5)
                    scrollphathd.flip(x=True, y=False)       
                    scrollphathd.show()
               
                except Exception as e:
                    print(e)
                    scrollphathd.clear()
                    scrollphathd.show()
            return "",204
        except Exception as e:
            print(e)
            abort(501)





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
_thread.start_new_thread(submit_td, (ip_addr, TD_DIRECTORY_ADDRESS))

# Run app server
app.run(host='0.0.0.0', port=8080)
