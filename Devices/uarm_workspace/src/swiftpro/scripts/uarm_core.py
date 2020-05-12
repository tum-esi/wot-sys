#!/usr/bin/env python3


# Import system library
import sys
import time
import rospy

# Import uarm for python library
from uarm.swift import Swift    

# Import messages type
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from std_msgs.msg import Int32
from swiftpro.msg import angle4th
from swiftpro.msg import position
from swiftpro.msg import status
from swiftpro.msg import SwiftproState


# from ..comm import Serial 



# move to function once received data from topic
def moveToCallback(position,sw):
	x = position.x
	y = position.y
	
	#if y>0:
	#	y = -y
	
	z = position.z
	print("Im moving to {},{},{}".format(x,y,z))
	sw.set_position(x, y, z, speed=1000)   
	


# pump control function once received data from topic
def pumpCallack(SwiftproState,sw):

	data_input = SwiftproState.pump

	#print('Gripping, data_input={}'.format(data_input))

	if data_input == 0:
		print('pump Off')
		sw.set_pump(on=False, wait=False)


	elif data_input == 1:
		print('pump on')
		sw.set_pump(on=True, wait=True, check=True)  #adjust wait to false and add timeout

	else:
		pass



def buzzerCallback(beep,sw):

	data_input = beep.data
	print('beep, data_input={}'.format(data_input))
	
	if data_input == 0:
		print('not beeping')
		sw.set_buzzer(wait= False)

	elif data_input == 1:
		print('start beeping')
		sw.set_buzzer(frequency=1000, duration=0.5, wait=True)

	else:
		pass
	


def listener():
	#print ' '
	#print 'Begin monitor mode - listening to all fucntional topics'
	
	print("===== Start creating node =====")
	sw = Swift(port='/dev/ttyACM0',timeout=20)
	rospy.init_node('swiftpro_write_node',anonymous=True)
	rospy.Subscriber("pump_state",SwiftproState, pumpCallack,sw)
	rospy.Subscriber("move_to",position, moveToCallback,sw)	
	#print("===== Finished initializing node =====")
	#rospy.Subscriber("uarm_status",String, attchCallback)
	rospy.Subscriber("buzzer_state",UInt8, buzzerCallback,sw)
	#rospy.Subscriber("pump_str_control",String, pumpStrCallack)
	#rospy.Subscriber("read_coords",Int32, currentCoordsCallback)
	#rospy.Subscriber("read_angles",Int32, readAnglesCallback)
	#rospy.Subscriber("stopper_status",Int32, stopperStatusCallback)
	#rospy.Subscriber("write_angles",Angles, writeAnglesCallback)
	rospy.spin()
	pass

if __name__ == '__main__':
	'''
	print("Im moving!!!")
	sw = Swift()
	for x in range(10):
		sw.set_position(90,90,10,10)
	'''
	print("===== Processing listener() =====")
	listener()


