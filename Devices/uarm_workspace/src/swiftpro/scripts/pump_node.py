#!/usr/bin/env python3


# Import system library
import sys
import rospy
import time

# Import uarm for python library
from uarm.swift import Swift

# Import messages type
from swiftpro.msg import position
from swiftpro.msg import SwiftproState
from std_msgs.msg import UInt8

#publisher
pub_1 = rospy.Publisher('pump_state',SwiftproState,queue_size = 10)
pub_2 = rospy.Publisher('move_to',position,queue_size = 10)
pub_3 = rospy.Publisher('buzzer_state',UInt8,queue_size = 10)
rospy.init_node('pump_node',anonymous = True)
msg1 = SwiftproState()
msg2 = position()
msg3 = position()
msg4 = position()
msg5 = position()

#input of messages
msg1.pump = 0

msg2.x = float(120)
msg2.y = float(180)
msg2.z = float(60)

msg3.x = float(120)
msg3.y = float(180)
msg3.z = float(3)

msg4.x = float(120)
msg4.y = float(-180)
msg4.z = float(60)

msg5.x = float(120)
msg5.y = float(-180)
msg5.z = float(3)

beep = 1

rate = rospy.Rate(10)

#Movements
print("=========start publishing pos1=========")
duration = 0
start_time = time.time()
while duration <= 5:
	pub_2.publish(msg2)
	duration = time.time()-start_time
print("=========finish publishing pos1=========")
rospy.sleep(10)

print("=========start publishing pos2=========")
duration2 = 0
second_time = time.time()
while duration2 <= 5:
	pub_2.publish(msg3)
	duration2 = time.time()-second_time
print("=========finish publishing pos2=========")
rospy.sleep(4)

print("=========pump on=========")
msg1.pump = 1
pub_1.publish(msg1)
rospy.sleep(3)

print("=========start publishing pos3=========")
duration3 = 0
third_time = time.time()	
while duration3 <= 5:
	pub_2.publish(msg4)
	duration3 = time.time()-third_time
print("=========finish publishing pos3=========")
rospy.sleep(15)

print("=========start publishing pos4=========")
duration4 = 0
forth_time = time.time()	
while duration4 <= 5:
	pub_2.publish(msg5)
	duration4 = time.time()-forth_time
print("=========finish publishing pos4=========")
rospy.sleep(4)

print("=========pump off=========")
msg1.pump = 0
pub_1.publish(msg1)
rospy.sleep(3)

print("=========go back to pos3=========")
duration5 = 0
fifth_time = time.time()
while duration5 <= 5:
	pub_2.publish(msg4)
	duration5 = time.time()-fifth_time
print("=========at initial pos3=========")
rospy.sleep(4)

print("=========beep=========")
pub_3.publish(beep)


rate.sleep()
	
	
