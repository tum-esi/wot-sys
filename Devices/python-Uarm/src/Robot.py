#! /usr/bin/env python3

import os
import sys
import time
import threading
import json


from uarm.swift import Swift
from flask import Flask, request, jsonify, json, abort
#from swiftpro.msg import SwiftproState
#from swiftpro.msg import position
#from swiftpro.msg import rotation
#from std_msgs.msg import UInt8

Position = 0



home_pos_x = 128.58
home_pos_y = 0
home_pos_z = 19.72

def moveToCallback(position,sw):
    #x = position.x
    #y = position.y
    #z = position.z
    #sw.set_position(x, y, z, speed=3000)
    return()

def moveWithSpeed(position,sw):
    return ()
    #pos_x = position.x
    #pos_y = position.y
    #pos_z = position.z
    #pos_sp = position.speed
    #sw.set_position(pos_x, pos_y, pos_z, speed=int(pos_sp))


def rotationCallback(rotation,sw):
    return()
    #stretch = rotation.stretch
    #rot = rotation.rotation
    #hgt = rotation.height
    #sw.set_polar(stretch=stretch, rotation=rot, height=hgt, speed=1000)

def gripCallack(SwiftproState,sw):
    return()
    #data_input = SwiftproState.gripper
    #if data_input == 0:
    #    sw.set_gripper(catch=False, timeout=2, wait=False)
    #elif data_input == 1:
    #    sw.set_gripper(catch=True, timeout=2, wait=False)
    #else:
    #    pass

def buzzerCallback(beep,sw):
    return()
    #data_input = beep.data
    #if data_input == 0:
    #    sw.set_buzzer(wait= False)
    #elif data_input == 1:
    #    sw.set_buzzer(frequency=500, duration=1, wait=True)
    #else:
    #    pass

def positionCallback(location,sw):
    return()
    #if location.data == 1:
    #    pos = sw.get_position()
    #    global Position
    #    Position = pos
    #else:
    #    pass

sw = Swift(port='/dev/ttyACM0',timeout=1)

def ROS_beep():
    sw.set_buzzer(frequency=500, duration=1, wait=True)
    return ()

def ROS_getlocation():
    pos = sw.get_position()
    return(pos)

def ROS_beepwithtime(payload):
    beep_duration = payload
    sw.set_buzzer(frequency=500, duration= beep_duration, wait=True)

def ROS_gohome():
    sw.set_position(home_pos_x, home_pos_y, home_pos_z, speed=3000)


def ROS_turnleft():
    [x,y,z] = ROS_getlocation()
    y_new = y + 1
    sw.set_position(x, y_new, z, speed=3000)


def ROS_turnright():
    [x,y,z] = ROS_getlocation()
    y_new = y - 1
    sw.set_position(x, y_new, z, speed=3000)

def ROS_goto(msg):
    sw.set_position(msg['x'], msg['y'], msg['z'], speed=3000)


def ROS_gowithspeed(msg):
    sw.set_position(msg['x'], msg['y'], msg['z'], speed= msg['speed'])

def ROS_gripaction(msg):
    sw.set_gripper(catch=msg, timeout=2, wait=False)