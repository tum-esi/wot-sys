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

def moveToCallback(position,sw):
    x = position.x
    y = position.y
    z = position.z
    sw.set_position(x, y, z, speed=3000)

def moveWithSpeed(position,sw):
    pos_x = position.x
    pos_y = position.y
    pos_z = position.z
    pos_sp = position.speed
    sw.set_position(pos_x, pos_y, pos_z, speed=int(pos_sp))


def rotationCallback(rotation,sw):
    stretch = rotation.stretch
    rot = rotation.rotation
    hgt = rotation.height
    sw.set_polar(stretch=stretch, rotation=rot, height=hgt, speed=1000)

def gripCallack(SwiftproState,sw):
    data_input = SwiftproState.gripper
    if data_input == 0:
        sw.set_gripper(catch=False, timeout=2, wait=False)
    elif data_input == 1:
        sw.set_gripper(catch=True, timeout=2, wait=False)
    else:
        pass

def buzzerCallback(beep,sw):
    data_input = beep.data
    if data_input == 0:
        sw.set_buzzer(wait= False)
    elif data_input == 1:
        sw.set_buzzer(frequency=500, duration=1, wait=True)
    else:
        pass

threading.Thread(target=lambda: rospy.init_node('gripper_node', disable_signals=True)).start()
sw = Swift(port='/dev/ttyACM0',timeout=20)
rospy.Subscriber("move_to",position,moveToCallback,sw)
rospy.Subscriber('move_speed',position,moveWithSpeed,sw)
rospy.Subscriber("turn",rotation,rotationCallback,sw)
rospy.Subscriber("grip_state",SwiftproState, gripCallack,sw)
rospy.Subscriber("buzzer_state",UInt8, buzzerCallback,sw)
pub1 = rospy.Publisher('move_to',position,queue_size = 10)
pub2 = rospy.Publisher('grip_state',SwiftproState,queue_size = 10)
pub3 = rospy.Publisher('buzzer_state',UInt8,queue_size = 10)
pub4 = rospy.Publisher('turn',rotation,queue_size = 10)
pub5 = rospy.Publisher('move_speed',position,queue_size = 10)

pos_x = 128.58
pos_y = 0
pos_z = 19.72
ip_adress= "192.168.0.112:8080"
TD=get_td(ip_adress)

app = Flask(__name__)


@app.route('/')
def thing_description():
    return json.dumps(TD), {'Content-Type':'application/json'}

@app.route('/actions/beep', methods=["post"])
def beep():
    rate = rospy.Rate(1)
    beep = 1
    beep_duration = 1
    beep_start = time.time()
    while time.time() < beep_start + beep_duration:
        pub3.publish(beep)
        rate.sleep()
        return jsonify("beeping")

@app.route('/actions/beepwithtime', methods=["POST"])
def beepwithtime():
    if request.is_json:
        
        schema=TD["actions"]["beepwithtime"]["input"]
        valid_input= Draft6Validator(schema).is_valid(request.json)
        print(request.json)
        print(valid_input)
        
        if valid_input:
            beep_duration = request.json
            timeout = request.json
            rate = rospy.Rate(1)
            beep = 1
            beep_start = time.time()
            while time.time() < beep_start + beep_duration:
                pub3.publish(beep)
                rate.sleep()
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
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub1.publish(msg)
        rate.sleep()
        print ("going Home")
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
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub4.publish(msg)
        rate.sleep()
        return jsonify("going left")

@app.route('/actions/turnright', methods=["POST"])
def turnright():
    msg = rotation()
    msg.stretch = float(168.29)
    msg.rotation = float(135)
    msg.height = float(42)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub4.publish(msg)
        rate.sleep()
        return jsonify("going right")

@app.route('/actions/goto', methods=["POST"])
def goto():
    if request.is_json:
        
        schema=TD["actions"]["goto"]["input"]
        valid_input= Draft6Validator(schema).is_valid(request.json)
        
        print(valid_input)
        
        if valid_input:
        
            msg = position()
            data = request.get_data()
            json_data = json.loads(data)
            msg.x = json_data['x']
            msg.y = json_data['y']
            msg.z = json_data['z']
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                pub1.publish(msg)
                rate.sleep()
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
        
        print(valid_input)
        
        if valid_input:
            msg = position()
            data = request.get_data()
            json_data = json.loads(data)
            msg.x = json_data['x']
            msg.y = json_data['y']
            msg.z = json_data['z']
            msg.speed = json_data['speed']
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                pub5.publish(msg)
                rate.sleep()
                return ("",204)
        else:
            abort(400)
    else:
        abort(415)

@app.route('/actions/grip', methods=["post"])
def grip():
    rate = rospy.Rate(10)
    msg = position()
    msg.x = float(120)
    msg.y = float(-180)
    msg.z = float(60)
    duration = 0
    start_time = time.time()
    while duration <= 5:    
        pub1.publish(msg)
        duration = time.time() - start_time
    rospy.sleep(8)

    msg2 = position()
    msg2.x = float(120)
    msg2.y = float(-180)
    msg2.z = float(10)
    duration2 = 0
    second_time = time.time()
    while duration2 <= 5:
        pub1.publish(msg2)
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
   
    msg3 = SwiftproState()
    msg3.gripper = 1
    
    rate = rospy.Rate(10)
    
    duration2 = 0
    second_time = time.time()
    while duration2 <= 5:
        pub2.publish(msg3)
        duration2 = time.time() - second_time
    
    rospy.sleep(6)
    rate.sleep()
    return jsonify("marcus1 works")

@app.route('/actions/gripopen', methods=["POST"])
def gripopen():
    
    msg3 = SwiftproState()
    msg3.gripper = 0
    
    rate = rospy.Rate(10)
    duration2 = 0
    second_time = time.time()
    while duration2 <= 5:
        pub2.publish(msg3)
        duration2 = time.time() - second_time
    
    rospy.sleep(6)
    rate.sleep()
    return jsonify("marcus1 works")
        
@app.route('/actions/marcus', methods=["GET", "PUT"])
def marcus():
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
                return jsonify("already set new home location")
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
    
@app.route('/actions/gripanddrop', methods=["POST"])
def gripanddrop():
    rate = rospy.Rate(10)
    msg = position()
    msg.x = float(120)
    msg.y = float(-180)
    msg.z = float(60)
    duration = 0
    start_time = time.time()
    while duration <= 5:
        pub1.publish(msg)
        duration = time.time() - start_time
    rospy.sleep(8)

    msg2 = position()
    msg2.x = float(120)
    msg2.y = float(-180)
    msg2.z = float(10)
    duration2 = 0
    second_time = time.time()
    while duration2 <= 5:
        pub1.publish(msg2)
        duration2 = time.time() - second_time
    rospy.sleep(6)

    msg3 = SwiftproState()
    msg3.gripper = 1
    pub2.publish(msg3)
    rospy.sleep(2)
    
    msg4 = position()
    msg4.x = float(128.58)
    msg4.y = float(0)
    msg4.z = float(19.72)
    duration4 = 0
    forth_time = time.time()
    while duration4 <=5:
        pub1.publish(msg4)
        duration4 = time.time() - forth_time
    rospy.sleep(8)

    msg5 = SwiftproState()
    msg3.gripper = 0
    pub2.publish(msg5)
    rospy.sleep(2)
    rate.sleep()
    return jsonify("finish")

if __name__ == '__main__':
    app.run(host=os.environ['ROS_IP'], port=8080)




