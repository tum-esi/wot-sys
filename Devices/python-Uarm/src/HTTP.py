#! /usr/bin/env python3

import os
import sys
import time
import threading
import json
import Robot

from uarm.swift import Swift
from flask import Flask, request, jsonify, json, abort

from jsonschema import Draft6Validator
from Robot import *

TD = 0

app = Flask(__name__)

# fct returns TD
@app.route('/')
def thing_description():
    return json.dumps(TD), {'Content-Type':'application/json'}

# fct lets Uarm beep 1s
@app.route('/uarm/actions/beep', methods=["POST"])
def beep():
    ROS_beep()
    return ("",204)

# fct lets Uarm beep 1s-3s
@app.route('/uarm/actions/beepwithtime', methods=["POST"])
def beepwithtime():
    if request.is_json:

        schema=TD["actions"]["beepWithTime"]["input"]
        valid_input= Draft6Validator(schema).is_valid(request.json)

        if valid_input:
            ROS_beepwithtime(request.json)
            return ("",204)
        else:
            abort(400)
    else:
        abort(415)

# fct returns Uarm to its home location
@app.route('/uarm/actions/gohome', methods=["POST"])
def gohome():
    ROS_gohome()
    return ("",204)


# fct sets or gets Uarm home location
@app.route('/uarm/properties/homeloc', methods=["GET", "PUT"])
def homeloc():

    if request.method == "PUT":
        if request.is_json:

            schema=TD["properties"]["homeLoc"]
            valid_input= Draft6Validator(schema).is_valid(request.json)

            print(valid_input)

            if valid_input:

                data = request.get_data()
                json_data = json.loads(data)

                Robot.home_pos_x = json_data['x']

                Robot.home_pos_y = json_data['y']

                Robot.home_pos_z = json_data['z']

                return ("",204)
            else:
                abort(400)
        else:
            abort(415)
    else:
        return_object ={
            "x": Robot.home_pos_x,
            "y": Robot.home_pos_y,
            "z": Robot.home_pos_z
            }
        return json.dumps(return_object), {'Content-Type':'application/json'}

#fct returns current location
@app.route('/uarm/properties/location', methods=["GET"])
def getlocation():

    pos = ROS_getlocation()
    return (jsonify(pos))

# fct turns Uarm left to a set position
@app.route('/uarm/actions/turnleft', methods=["POST"])
def turnleft():
    [x,y,z] = ROS_getlocation()
    y_new = y + 1
		
    schema=TD["actions"]["turnLeft"]["input"]["properties"]["y"]
    valid_input= Draft6Validator(schema).is_valid(y_new)
			
    if valid_input:
        #ROS
        ROS_turnleft()
        return ("",204)
    else:
        print("Reached Max Limit")
        abort(400)

# fct turns Uarm right to a set position
@app.route('/uarm/actions/turnright', methods=["POST"])
def turnright():
    [x,y,z] = ROS_getlocation()
    y_new = y - 1
		
    schema=TD["actions"]["turnRight"]["input"]["properties"]["y"]
    valid_input= Draft6Validator(schema).is_valid(y_new)
			
    if valid_input:
        #ROS
        ROS_turnright()
        return ("",204)
    else:
        print("Reached Max Limit")
        abort(400)

# fct tells Uarm zu go to given position
@app.route('/uarm/actions/goto', methods=["POST"])
def goto():
    if request.is_json:

        schema=TD["actions"]["goTo"]["input"]
        valid_input= Draft6Validator(schema).is_valid(request.json)

        if valid_input:

            msg = {'x': 0, 'y': 0, 'z': 0}
            data = request.get_data()
            json_data = json.loads(data)
            msg['x'] = json_data['x']
            msg['y'] = json_data['y']
            msg['z'] = json_data['z']
            ROS_goto(msg)
            return ("",204)
        else:
             abort(400)
    else:
         abort(415)

# fct tells Uarm zu go to given position with an additional variable for speed
@app.route('/uarm/actions/gowithspeed', methods=["POST"])
def gowithspeed():
    if request.is_json:

        schema=TD["actions"]["goWithSpeed"]["input"]
        valid_input= Draft6Validator(schema).is_valid(request.json)

        if valid_input:
            msg = {'x': 0, 'y': 0, 'z': 0}
            data = request.get_data()
            json_data = json.loads(data)
            msg['x'] = json_data['x']
            msg['y'] = json_data['y']
            msg['z'] = json_data['z']
            msg['speed'] = json_data['speed']
            ROS_gowithspeed(msg)
            return ("",204)
        else:
            abort(400)
    else:
        abort(415)


# fct closes Uarm gripper
@app.route('/uarm/actions/gripclose', methods=["POST"])
def gripclose():

    msg = True
    ROS_gripaction(msg)
    return ("",204)


# fct opens Uarm gripper
@app.route('/uarm/actions/gripopen', methods=["POST"])
def gripopen():

    msg = False
    ROS_gripaction(msg)
    return ("",204)



def push_HTTP_TD(TD_HTTP):
    global TD
    TD = TD_HTTP

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('gripper_HTTP_node', disable_signals=True)).start()
    app.run(host=os.environ['ROS_IP'], port=8080)



