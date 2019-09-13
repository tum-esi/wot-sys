#! /usr/bin/env python3

import os
import sys
import rospy
import time
import threading

#Import uarm for python library
from uarm.swift import Swift
from flask import Flask, request, jsonify, json, abort
# Import messages type
from swiftpro.msg import SwiftproState
from swiftpro.msg import position
from swiftpro.msg import rotation
from std_msgs.msg import UInt8

#Subscriber callbackfunction
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

#Set a thread for ros
threading.Thread(target=lambda: rospy.init_node('gripper_node', disable_signals=True)).start()
sw = Swift(port='/dev/ttyACM0',timeout=20)

#define subscriber
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

#Set original home position
pos_x = 128.58
pos_y = 0
pos_z = 19.72


app = Flask(__name__)

#publisher
@app.route('/actions/beep', methods=["post"])
def beep():
    rate = rospy.Rate(1)
    beep = 1
    timeout = 1
    timeout_start = time.time()
    while time.time() < timeout_start + timeout:
        pub3.publish(beep)
        rate.sleep()
        return jsonify("beeping")

@app.route('/actions/beepwithtime', methods=["POST"])
def beepwithtime():
    if request.is_json:
        timeout = request.json
        if timeout <= 3:
            print(timeout)
            rate = rospy.Rate(1)
            beep = 1
            timeout_start = time.time()
            while time.time() < timeout_start + timeout:
                pub3.publish(beep)
                rate.sleep()
            return jsonify("beeping with time")
        else:
            return jsonify("please input a smaller beep duration between 1 and 3")
    else:
        return jsonify("error")

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
        return jsonify("going home")

@app.route('/properties/homeloc', methods=["GET", "PUT"])  
def homeloc():
    if request.method == "PUT":
        if request.is_json:
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
        return(pos_x, pos_y, pos_z)

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
            return jsonify("moving")
    else:
        return jsonify("error")

@app.route('/actions/gowithspeed', methods=["POST"])
def gowithspeed():
    if request.is_json:
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
            return jsonify("moving")
    else:
        return jsonify("error")

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
    app.run(host=os.environ['ROS_IP'], port=5000)




