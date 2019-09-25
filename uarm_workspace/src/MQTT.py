
#!/usr/bin/env python3
import paho.mqtt.client as mqtt

import os
import sys
import rospy
import time
import threading
import json


from uarm.swift import Swift

from swiftpro.msg import SwiftproState
from swiftpro.msg import position
from swiftpro.msg import rotation
from std_msgs.msg import UInt8

from Uarm_TD import get_td
from jsonschema import Draft6Validator
from ast import literal_eval

pos_x = 128.58
pos_y = 0
pos_z = 19.72
ip_adress= "192.168.0.112:8080"
TD=get_td(ip_adress)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    client.subscribe("#")

def on_message(client, userdata, msg):
	#print(msg.topic + " " + str(msg.payload.decode()))
	parserMQTTtopic(msg.topic, str(msg.payload.decode()))
	
    
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

def parserMQTTtopic(MQTT_topic,MQTT_payload):
	
	#print(MQTT_topic)
	
	if MQTT_topic == "/actions/beep":
		
		parserMQTTpayload(MQTT_payload)
		rate = rospy.Rate(1)
		beep = 1
		beep_duration = 1
		beep_start = time.time()
		while time.time() < beep_start + beep_duration:
			pub3.publish(beep)
			rate.sleep()
			
	elif MQTT_topic == "/actions/beepwithtime":
		
		json_flag = True
		
		#check if payload is json format
		try:
			json.loads(MQTT_payload)
		except ValueError:
			json_flag = False
		
		if json_flag:
			#turn payload from string in dict
			dict_payload = literal_eval(MQTT_payload)
			
			#compare TD with payload dict
			schema=TD["actions"]["beepwithtime"]["input"]	
			valid_input= Draft6Validator(schema).is_valid(dict_payload)
			
			if valid_input:
				#ROS
				beep_duration = dict_payload
				
				rate = rospy.Rate(1)
				beep = 1
				beep_start = time.time()
				while time.time() < beep_start + beep_duration:
					pub3.publish(beep)
					rate.sleep()
			else:
				print("Wrong Input")
				
		else:
			print("NO Json")
			
			
	elif MQTT_topic == "/actions/gohome":
		
		msg_home = position()
		msg_home.x = pos_x
		msg_home.y = pos_y
		msg_home.z = pos_z
		rate = rospy.Rate(10)
		
		pub1.publish(msg_home)
		rate.sleep()
		
			
	elif MQTT_topic == "/actions/turnleft":
		print(MQTT_topic)
	elif MQTT_topic == "/actions/turnright":
		print(MQTT_topic)
	elif MQTT_topic == "/actions/goto":
		
		json_flag = True
		
		#check if payload is json format
		try:
			json.loads(MQTT_payload)
		except ValueError:
			json_flag = False
		
		if json_flag:
			#turn payload from string in dict
			dict_payload = literal_eval(MQTT_payload)
			
			#compare TD with payload dict
			schema=TD["actions"]["goto"]["input"]	
			valid_input= Draft6Validator(schema).is_valid(dict_payload)
			
			if valid_input:
				#ROS
				msg = position()
				
				json_data = json.loads(MQTT_payload)
				
				msg.x = json_data['x']
				msg.y = json_data['y']
				msg.z = json_data['z']
				rate = rospy.Rate(10)
				
				while not rospy.is_shutdown():
					pub1.publish(msg)
					rate.sleep()
					return ("",204)
			else:
				print("Wrong Input")
				
		else:
			print("NO Json")
		
	elif MQTT_topic == "/actions/gowithspeed":
		#speed wont set
		json_flag = True
		
		#check if payload is json format
		try:
			json.loads(MQTT_payload)
		except ValueError:
			json_flag = False
		
		if json_flag:
			#turn payload from string in dict
			dict_payload = literal_eval(MQTT_payload)
			
			#compare TD with payload dict
			schema=TD["actions"]["gowithspeed"]["input"]	
			valid_input= Draft6Validator(schema).is_valid(dict_payload)
			
			if valid_input:
				#ROS
				msg = position()
				
				json_data = json.loads(MQTT_payload)
				
				msg.x = json_data['x']
				msg.y = json_data['y']
				msg.z = json_data['z']
				msg.speed = json_data['speed']
				print(msg)
				rate = rospy.Rate(10)
				# what is this while not shut down ?
				while not rospy.is_shutdown():
					pub5.publish(msg)
					rate.sleep()
					return ("",204)
			else:
				print("Wrong Input")
				
		else:
			print("NO Json")
		
	elif MQTT_topic == "/actions/grip":
		print(MQTT_topic)
	elif MQTT_topic == "/actions/gripclose":
		
		msg3 = SwiftproState()
		msg3.gripper = 1
		
		rate = rospy.Rate(10)
		
		duration2 = 0
		second_time = time.time()
		while duration2 <= 5:
			pub2.publish(msg3)
			duration2 = time.time() - second_time
		
		rospy.sleep(2)
		rate.sleep()
		
		
	elif MQTT_topic == "/actions/gripopen":
		
		msg3 = SwiftproState()
		msg3.gripper = 0
		
		rate = rospy.Rate(10)
		duration2 = 0
		second_time = time.time()
		while duration2 <= 5:
			pub2.publish(msg3)
			duration2 = time.time() - second_time
		
		rospy.sleep(2)
		rate.sleep()
		
		
	elif MQTT_topic == "/actions/marcus":
		
		json_flag = True
		
		try:
			json.loads(MQTT_payload)
		except ValueError:
			json_flag = False
		
		if json_flag:
			test3 = literal_eval(MQTT_payload)
			
			schema=TD["actions"]["beepwithtime"]["input"]
				
			valid_input= Draft6Validator(schema).is_valid(test3)
			
			if valid_input:
				
				beep_duration = test3
				
				rate = rospy.Rate(1)
				beep = 1
				beep_start = time.time()
				while time.time() < beep_start + beep_duration:
					pub3.publish(beep)
					rate.sleep()
			else:
				print("Wrong Input")
				return 0
		else:
			print("NO Json")
		
	elif MQTT_topic == "/actions/gripanddrop":
		print(MQTT_topic)
	else:
		print("WrongCMD")
		
	
	

def parserMQTTpayload(MQTT_payload):
	print(MQTT_payload)

	

threading.Thread(target=lambda: rospy.init_node('gripper_MQTT_node', disable_signals=True)).start()
#Next line in Robot.py
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
ip_adress= "192.168.0.112:1883"
TD=get_td(ip_adress)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("192.168.0.116", 1883, 60)

client.loop_forever()
