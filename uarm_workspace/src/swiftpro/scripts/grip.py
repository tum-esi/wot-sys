#!/usr/bin/env python3

import rospy
import sys

from uarm.swift import Gripper
from swiftpro.msg import SwiftproState
from std_msgs.msg import UInt8


def raiseError():
	print('ERROR: Input Incorrect')
	print('ERROR: Input off / 0  or  on / 1')


def execute():
	pub_2 = rospy.Publisher('grip_state',SwiftproState,queue_size = 10)
	rospy.init_node('grip_node',anonymous = True)
	rate = rospy.Rate(1)
	msg = SwiftproState()
	msg.gripper = 0


	while not rospy.is_shutdown():
		print("=========Publishing=========")
		rospy.loginfo(msg)
		pub_2.publish(msg)
		rate.sleep()



if __name__ == '__main__':
	try:
		execute()
	except:
		print('==========================================')
		print('ERROR: exectuion error')
		raiseError()
		pass
