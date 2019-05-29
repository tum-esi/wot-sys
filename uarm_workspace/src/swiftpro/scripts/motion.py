#!/usr/bin/env python3


import rospy
import sys
#from swiftpro.msg import angle4th
from swiftpro.msg import position
#from swiftpro.msg import status
#from swiftpro.msg import SwiftproState


# main exection function
def execute():
	print("========We start creating publisher=========")
	# define publisher and its topic 
	pub_1 = rospy.Publisher('move_to',position,queue_size = 10)

	print("=========We finished creating publisher=========")

	rospy.init_node('motion_node',anonymous = True)

	print("=========We finished initializing node=========")

	rate = rospy.Rate(10)

	# input x,y,z
	msg = position()
	msg.x = float(10)
	msg.y = float(150)
	if msg.y> 0:
		msg.y = -msg.y
	msg.z = float(60)

	while not rospy.is_shutdown():
		print("=========Publishing=========")
		pub_1.publish(msg)
		rate.sleep()



# main function
if __name__ == '__main__':
	execute()
