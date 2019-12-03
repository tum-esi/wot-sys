
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

from Robot import *
from jsonschema import Draft6Validator
from ast import literal_eval

msg_home = position()
msg_home.x = 128.58
msg_home.y = 0
msg_home.z = 19.72

TD = 0

# Topic strings
beep = "/actions/beep"
beepWT = "/actions/beepwithtime"
gohome = "/actions/gohome"
turnleft = "/actions/turnleft"
turnright = "/actions/turnright"
goto = "/actions/goto"
gowithspeed = "/actions/gowithspeed"
sequence1 = "/actions/sequence1"
gripclose = "/actions/gripclose"
gripopen = "/actions/gripopen"
location = "/properties/location"
homeloc = "/properties/homeloc"
homeloc_write ="/properties/homeloc/writeproperty"
Thing_Description = "/Uarm_TD"

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
	
	#subscribe to all cmd channels
    client.subscribe(beep)
    client.subscribe(beepWT)
    client.subscribe(gohome)
    client.subscribe(turnleft)
    client.subscribe(turnright)
    client.subscribe(goto)
    client.subscribe(gowithspeed)
    client.subscribe(sequence1)
    client.subscribe(gripclose)
    client.subscribe(gripopen)
    client.subscribe(location)
    client.subscribe(homeloc_write)
    client.publish(Thing_Description,json.dumps(TD),retain=True)
    client.publish(homeloc,json.dumps(msg_home.x),retain=True)

def on_message(client, userdata, msg):
	#print(msg.topic + " " + str(msg.payload.decode()))
	parserMQTTtopic(msg.topic, str(msg.payload.decode()),client)
	
    

def parserMQTTtopic(MQTT_topic,MQTT_payload,client):
	
	# lets Uarm beep 1s
	if MQTT_topic == beep:
		
		parserMQTTpayload(MQTT_payload)
		ROS_beep()
	
	# sets new home location
	elif MQTT_topic == homeloc_write:
		
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
			schema=TD["properties"]["homeloc"]	
			valid_input= Draft6Validator(schema).is_valid(dict_payload)
	
			if valid_input:
				
				json_data = json.loads(MQTT_payload)
				
				msg_home.x = json_data['x']
				msg_home.y = json_data['y']
				msg_home.z = json_data['z']
				client.publish(homeloc,json.dumps(str(msg_home)),retain=True)
				
			else:
				print("Wrong Input")
				
		else:
			print("NO Json")
		
	# lets Uarm beep 1s-3s
	elif MQTT_topic == beepWT:
		
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
			
	# returns Uarm to its home location		
	elif MQTT_topic == gohome:
		
		ROS_gohome(msg_home)
		
	# turns Uarm left to a set position		
	elif MQTT_topic == turnleft:
		
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
		else:
			print("Reached Max Limit")
		
	
	# turns Uarm right to a set position
	elif MQTT_topic == turnright:
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
		else:
			print("Reached Max Limit")
	
	# tells Uarm zu go to given position
	elif MQTT_topic == goto:
		
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
	
	# tells Uarm zu go to given position with an additional variable for speed
	elif MQTT_topic == gowithspeed:
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
	
	# for a specific sequence of steps. Uarm grips at a certain position
	elif MQTT_topic == sequence1:
		
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
		
		print("Works")

	
	# closes Uarm gripper
	elif MQTT_topic == gripclose:
		
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
		
	# opens Uarm gripper	
	elif MQTT_topic == gripopen:
		
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
		
	# prints location in terminal	
	elif MQTT_topic == location:
		
		[x,y,z] = ROS_getlocation()
		print(x,y,z)
		msg_location = position()
		msg_location.x = x
		msg_location.y = y
		msg_location.z = z
		#triggers itself -.-
		#client.publish(location,json.dumps(msg_location.x),retain=True)
			

		
	else:
		print("WrongCMD")
			

def parserMQTTpayload(MQTT_payload):
	print(MQTT_payload)

def main_MQTT(TD_MQTT):
	global TD
	TD = TD_MQTT
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
