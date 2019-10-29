

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

sw = Swift(port='/dev/ttyACM0',timeout=1)
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

def ROS_beep():
    rate = rospy.Rate(1)
    beep = 1
    beep_duration = 1
    beep_start = time.time()
    while time.time() < beep_start + beep_duration:
        pub3.publish(beep)
        rate.sleep()
        return True

def ROS_beepwithtime(payload):
    beep_duration = payload
    rate = rospy.Rate(1)
    beep = 1
    beep_start = time.time()
    while time.time() < beep_start + beep_duration:
        pub3.publish(beep)
        rate.sleep()

def ROS_gohome(msg):
    rate = rospy.Rate(10)
    pub1.publish(msg)
    rate.sleep()

def ROS_turnleft(msg):
    rate = rospy.Rate(10)
    pub4.publish(msg)
    rate.sleep()

def ROS_turnright(msg):
    rate = rospy.Rate(10)
    pub4.publish(msg)
    rate.sleep()

def ROS_goto(msg):
    rate = rospy.Rate(10)
    pub1.publish(msg)
    rate.sleep()

def ROS_gowithspeed(msg):
    rate = rospy.Rate(10)
    pub5.publish(msg)
    rate.sleep()

def ROS_gripaction(msg):
    rate = rospy.Rate(10)
    pub2.publish(msg)
