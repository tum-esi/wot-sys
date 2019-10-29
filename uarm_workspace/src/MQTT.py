
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
from Robot import *
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
	
    

def parserMQTTtopic(MQTT_topic,MQTT_payload):
	
	#print(MQTT_topic)
	
	if MQTT_topic == "/actions/beep":
		
		parserMQTTpayload(MQTT_payload)
		ROS_beep()
			
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
				ROS_beepwithtime(dict_payload)
			else:
				print("Wrong Input")
				
		else:
			print("NO Json")
			
			
	elif MQTT_topic == "/actions/gohome":
		
		msg_home = position()
		msg_home.x = pos_x
		msg_home.y = pos_y
		msg_home.z = pos_z
		
		ROS_gohome(msg_home)
		
			
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
				
				
				while not rospy.is_shutdown():
					ROS_goto(msg)
					return ("",204)
			else:
				print("Wrong Input")
				
		else:
			print("NO Json")
		
	elif MQTT_topic == "/actions/gowithspeed":
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
				
				while not rospy.is_shutdown():
					ROS_gowithspeed(msg)
					return ("",204)
			else:
				print("Wrong Input")
				
		else:
			print("NO Json")
		
	elif MQTT_topic == "/actions/sequence1":
		print(MQTT_topic)
	elif MQTT_topic == "/actions/gripclose":
		
		msg3 = SwiftproState()
		msg3.gripper = 1
		
		rate = rospy.Rate(10)
		
		duration2 = 0
		second_time = time.time()
		while duration2 <= 5:
			ROS_gripaction(msg3)
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
			ROS_gripaction(msg3)
			duration2 = time.time() - second_time
		
		rospy.sleep(2)
		rate.sleep()
		
		
	else:
		print("WrongCMD")
		
	
	

def parserMQTTpayload(MQTT_payload):
	print(MQTT_payload)

def main_MQTT():
	client = mqtt.Client()
	client.on_connect = on_connect
	client.on_message = on_message

	client.connect("192.168.0.116", 1883, 60)

	client.loop_start()

if __name__ == '__main__':
	threading.Thread(target=lambda: rospy.init_node('gripper_HTTP_node', disable_signals=True)).start()
	client = mqtt.Client()
	client.on_connect = on_connect
	client.on_message = on_message

	client.connect("192.168.0.116", 1883, 60)

	client.loop_forever()
