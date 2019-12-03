#! /usr/bin/env python3

import os
import sys
import rospy
import time
import threading
import json


from uarm.swift import Swift
from flask import Flask, request, jsonify, json, abort
from swiftpro.msg import SwiftproState
from swiftpro.msg import position
from swiftpro.msg import rotation
from std_msgs.msg import UInt8

from jsonschema import Draft6Validator
from Robot import *


pos_x = 128.58
pos_y = 0
pos_z = 19.72
TD = 0

app = Flask(__name__)

# fct returns TD
@app.route('/')
def thing_description():
    return json.dumps(TD), {'Content-Type':'application/json'}

# fct lets Uarm beep 1s
@app.route('/actions/beep', methods=["POST"])
def beep():
    if ROS_beep():
        return ("",204)

# fct lets Uarm beep 1s-3s
@app.route('/actions/beepwithtime', methods=["POST"])
def beepwithtime():
    if request.is_json:
        
        schema=TD["actions"]["beepwithtime"]["input"]
        valid_input= Draft6Validator(schema).is_valid(request.json)
        
        if valid_input:
            ROS_beepwithtime(request.json)
            return ("",204)
        else:
            abort(400)
    else:
        abort(415)

# fct returns Uarm to its home location
@app.route('/actions/gohome', methods=["POST"])
def gohome():
    msg = position()
    msg.x = pos_x
    msg.y = pos_y
    msg.z = pos_z
    while not rospy.is_shutdown():
        ROS_gohome(msg)
        return ("",204)

# fct sets or gets Uarm home location
@app.route('/properties/homeloc', methods=["GET", "PUT"])  
def homeloc():
    
    if request.method == "PUT":
        if request.is_json:
            
            schema=TD["properties"]["homeloc"]
            valid_input= Draft6Validator(schema).is_valid(request.json)
        
            print(valid_input)
        
            if valid_input:
            
                data = request.get_data()
                json_data = json.loads(data)
                global pos_x
                pos_x = json_data['x']
                global pos_y
                pos_y = json_data['y']
                global pos_z
                pos_z = json_data['z']
                return ("",204)
            else:
                abort(400)
        else:
            abort(415)
    else:
        return_object ={
            "x": pos_x,
            "y": pos_y,
            "z": pos_z
            }
        return json.dumps(return_object), {'Content-Type':'application/json'}

#fct returns current location
@app.route('/properties/location', methods=["GET"])
def getlocation():
    
    pos = ROS_getlocation()
    return (jsonify(pos))
    
# fct turns Uarm left to a set position
@app.route('/actions/turnleft', methods=["POST"])
def turnleft():
    [x,y,z] = ROS_getlocation()
    y_new = y + 1
		
    msg = position()
    msg.x = x
    msg.y = y_new
    msg.z = z
		
    schema=TD["actions"]["turnleft"]["input"]["properties"]["y"]
    valid_input= Draft6Validator(schema).is_valid(y_new)
			
    if valid_input:
        #ROS
        ROS_goto(msg)
        return ("",204)
    else:
        print("Reached Max Limit")
        abort(400)
            
# fct turns Uarm right to a set position
@app.route('/actions/turnright', methods=["POST"])
def turnright():
    [x,y,z] = ROS_getlocation()
    y_new = y - 1
		
    msg = position()
    msg.x = x
    msg.y = y_new
    msg.z = z
		
    schema=TD["actions"]["turnright"]["input"]["properties"]["y"]
    valid_input= Draft6Validator(schema).is_valid(y_new)
			
    if valid_input:
        #ROS
        ROS_goto(msg)
        return ("",204)
    else:
        print("Reached Max Limit")
        abort(400)
            
# fct tells Uarm zu go to given position
@app.route('/actions/goto', methods=["POST"])
def goto():
    if request.is_json:
        
        schema=TD["actions"]["goto"]["input"]
        valid_input= Draft6Validator(schema).is_valid(request.json)
        
        if valid_input:
        
            msg = position()
            data = request.get_data()
            json_data = json.loads(data)
            msg.x = json_data['x']
            msg.y = json_data['y']
            msg.z = json_data['z']
            while not rospy.is_shutdown():
                ROS_goto(msg)
                return ("",204)
        else:
             abort(400)
    else:
         abort(415)
    
# fct tells Uarm zu go to given position with an additional variable for speed
@app.route('/actions/gowithspeed', methods=["POST"])
def gowithspeed():
    if request.is_json:
        
        schema=TD["actions"]["gowithspeed"]["input"]
        valid_input= Draft6Validator(schema).is_valid(request.json)
        
        if valid_input:
            msg = position()
            data = request.get_data()
            json_data = json.loads(data)
            msg.x = json_data['x']
            msg.y = json_data['y']
            msg.z = json_data['z']
            msg.speed = json_data['speed']
            while not rospy.is_shutdown():
                ROS_gowithspeed(msg)
                return ("",204)
        else:
            abort(400)
    else:
        abort(415)

# fct for a specific sequence of steps. Uarm grips at a certain position
@app.route('/actions/sequence1', methods=["POST"])
def sequence1():
    
    msg = position()
    msg.x = float(120)
    msg.y = float(-180)
    msg.z = float(60)
    duration = 0
    start_time = time.time()
    while duration <= 5:   
        ROS_goto(msg)
        duration = time.time() - start_time

    msg2 = position()
    msg2.x = float(120)
    msg2.y = float(-180)
    msg2.z = float(10)
    duration2 = 0
    second_time = time.time()
    while duration2 <= 5:
        ROS_goto(msg2)
        duration2 = time.time() - second_time

    msg3 = SwiftproState()
    msg3.gripper = 1
    pub2.publish(msg3)
    rospy.sleep(2)

    return jsonify("works")

# fct closes Uarm gripper
@app.route('/actions/gripclose', methods=["POST"])
def gripclose():
   
    msg = SwiftproState()
    msg.gripper = 1
    duration = 0
    second_time = time.time()
    
    while duration <= 4:
        ROS_gripaction(msg)
        duration = time.time() - second_time
    return ("",204)

# fct opens Uarm gripper
@app.route('/actions/gripopen', methods=["POST"])
def gripopen():
    
    msg = SwiftproState()
    msg.gripper = 0
    duration = 0
    second_time = time.time()
    
    while duration <= 4:
        ROS_gripaction(msg)
        duration = time.time() - second_time
    return ("",204)

def push_HTTP_TD(TD_HTTP):
    global TD
    TD = TD_HTTP

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('gripper_HTTP_node', disable_signals=True)).start()
    app.run(host=os.environ['ROS_IP'], port=8080)




