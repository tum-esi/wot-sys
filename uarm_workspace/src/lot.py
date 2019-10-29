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

from Uarm_TD import get_td
from jsonschema import Draft6Validator
from Robot import *



pos_x = 128.58
pos_y = 0
pos_z = 19.72

ip_adress= "192.168.0.112:8080"
TD=get_td(ip_adress)

app = Flask(__name__)


@app.route('/')
def thing_description():
    return json.dumps(TD), {'Content-Type':'application/json'}

@app.route('/actions/beep', methods=["POST"])
def beep():
    if ROS_beep():
        return jsonify("beeping")

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

@app.route('/actions/gohome', methods=["POST"])
def gohome():
    msg = position()
    msg.x = pos_x
    msg.y = pos_y
    msg.z = pos_z
    while not rospy.is_shutdown():
        ROS_gohome(msg)
        return jsonify("going home")

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

@app.route('/actions/turnleft', methods=["POST"])
def turnleft():
    msg = rotation()
    msg.stretch = float(168.29)
    msg.rotation = float(45)
    msg.height = float(42)
    
    while not rospy.is_shutdown():
        ROS_turnleft(msg)
        return jsonify("going left")

@app.route('/actions/turnright', methods=["POST"])
def turnright():
    msg = rotation()
    msg.stretch = float(168.29)
    msg.rotation = float(135)
    msg.height = float(42)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ROS_turnright(msg)
        return jsonify("going right")

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
    rospy.sleep(8)

    msg2 = position()
    msg2.x = float(120)
    msg2.y = float(-180)
    msg2.z = float(10)
    duration2 = 0
    second_time = time.time()
    while duration2 <= 5:
        ROS_goto(msg2)
        duration2 = time.time() - second_time
    rospy.sleep(6)

    msg3 = SwiftproState()
    msg3.gripper = 1
    pub2.publish(msg3)
    rospy.sleep(2)
    print("grip done!")
    rate.sleep()
    return jsonify("finish")

@app.route('/actions/gripclose', methods=["POST"])
def gripclose():
   
    msg = SwiftproState()
    msg.gripper = 1
    duration = 0
    second_time = time.time()
    
    while duration <= 4:
        ROS_gripaction(msg)
        duration = time.time() - second_time
    return jsonify("works")

@app.route('/actions/gripopen', methods=["POST"])
def gripopen():
    
    msg = SwiftproState()
    msg.gripper = 0
    duration = 0
    second_time = time.time()
    
    while duration <= 4:
        ROS_gripaction(msg)
        duration = time.time() - second_time
    return jsonify("works1")

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('gripper_HTTP_node', disable_signals=True)).start()
    app.run(host=os.environ['ROS_IP'], port=8080)




