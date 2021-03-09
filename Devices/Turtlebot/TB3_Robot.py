import rospy
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import moveit_commander
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import MoveGroupActionGoal
import threading
from flask import request
import time

#Init states TB3_Base
velocity_msg = Twist()
stop_vehicle = Twist()
#linear movement
velocity_msg.linear.x = 0.0
velocity_msg.linear.y = 0.0
velocity_msg.linear.z = 0.0
#angular movement
velocity_msg.angular.x = 0.0
velocity_msg.angular.y = 0.0
velocity_msg.angular.z = 0.0
#stop message
stop_vehicle.linear.x = 0.0
stop_vehicle.linear.y = 0.0
stop_vehicle.linear.z = 0.0
#angular movement
stop_vehicle.angular.x = 0.0
stop_vehicle.angular.y = 0.0
stop_vehicle.angular.z = 0.0
#Init gripper
gripper_msg = Float64MultiArray()

#ROS Channel
pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pub = rospy.Publisher('/move_group/goal', MoveGroupActionGoal, queue_size=10)
pub3 = rospy.Publisher('/gripper_position', Float64MultiArray, queue_size=10)
#threading.Thread(target=lambda: rospy.init_node('Turtel_Control', anonymous=False, log_level = None)).start()
#rospy.init_node('Turtel_Control', anonymous=False, log_level = None)

def init_robot():
    

    #Init ROS MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    group_variable_values = group.get_current_joint_values()
    print ("============ Joint values: ", group_variable_values)

    group_variable_values[0] = 0.0
    group_variable_values[1] = -1.0
    group_variable_values[2] = 0.9
    group_variable_values[3] = 0.0
    group.set_joint_value_target(group_variable_values)

    plan2 = group.plan()
    group.execute(plan2)

def base_move_forward():
    velocity_msg.linear.x = velocity_msg.linear.x + 0.1
    pub1.publish(velocity_msg)

def base_move_forward_with_time(seconds):
    velocity_msg.linear.x = velocity_msg.linear.x + 0.4
    pub1.publish(velocity_msg)
    time.sleep(seconds)
    pub1.publish(stop_vehicle)

def base_move_backward():
    velocity_msg.linear.x = velocity_msg.linear.x - 0.1
    pub1.publish(velocity_msg)

def base_move_backward_with_time(seconds):
    velocity_msg.linear.x = velocity_msg.linear.x - 0.4
    pub1.publish(velocity_msg)
    time.sleep(seconds)
    pub1.publish(stop_vehicle)

def base_rotate_left():
    velocity_msg.angular.z = velocity_msg.angular.z + 0.1
    pub1.publish(velocity_msg)

def base_rotate_left_with_time(seconds):
    velocity_msg.angular.z = velocity_msg.angular.z + 0.4
    pub1.publish(velocity_msg)
    time.sleep(seconds)
    pub1.publish(stop_vehicle)

def base_rotate_right():
    velocity_msg.angular.z = velocity_msg.angular.z - 0.1
    pub1.publish(velocity_msg)

def base_rotate_right_with_time(seconds):
    velocity_msg.angular.z = velocity_msg.angular.z - 0.4
    pub1.publish(velocity_msg)
    time.sleep(seconds)
    pub1.publish(stop_vehicle)

def stop_wheel():
    pub1.publish(stop_vehicle)

def arm_move():

    group = moveit_commander.MoveGroupCommander("arm")
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = group_variable_values[0] + 0.3
    group.set_joint_value_target(group_variable_values)
    plan4 = group.plan()
    group.execute(plan4)

def arm_front_position1():

    group = moveit_commander.MoveGroupCommander("arm")
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = 0.0
    group_variable_values[1] = 0.7
    group_variable_values[2] = -0.9
    group_variable_values[3] = 0.0
    group.set_joint_value_target(group_variable_values)

    plan3 = group.plan()
    group.execute(plan3)

def arm_front_position2():
    
    group = moveit_commander.MoveGroupCommander("arm")
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = 0.0
    group_variable_values[1] = 0.9
    group_variable_values[2] = -0.9
    group_variable_values[3] = 1.2
    group.set_joint_value_target(group_variable_values)

    plan3 = group.plan()
    group.execute(plan3)

def arm_front_position3():
    
    group = moveit_commander.MoveGroupCommander("arm")
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = 0.0
    group_variable_values[1] = 1.2
    group_variable_values[2] = -0.9
    group_variable_values[3] = 1.2
    group.set_joint_value_target(group_variable_values)

    plan3 = group.plan()
    group.execute(plan3)

def arm_left_position1():

    group = moveit_commander.MoveGroupCommander("arm")
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = 1.6
    group_variable_values[1] = 0.7
    group_variable_values[2] = -0.9
    group_variable_values[3] = 0.0
    group.set_joint_value_target(group_variable_values)

    plan3 = group.plan()
    group.execute(plan3)

def arm_left_position2():
    
    group = moveit_commander.MoveGroupCommander("arm")
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = 1.6
    group_variable_values[1] = 0.9
    group_variable_values[2] = -0.9
    group_variable_values[3] = 1.2
    group.set_joint_value_target(group_variable_values)

    plan3 = group.plan()
    group.execute(plan3)

def arm_left_position3():
    
    group = moveit_commander.MoveGroupCommander("arm")
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = 1.6
    group_variable_values[1] = 1.2
    group_variable_values[2] = -0.9
    group_variable_values[3] = 1.2
    group.set_joint_value_target(group_variable_values)

    plan3 = group.plan()
    group.execute(plan3)

def arm_right_position1():

    group = moveit_commander.MoveGroupCommander("arm")
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = -1.6
    group_variable_values[1] = 0.7
    group_variable_values[2] = -0.9
    group_variable_values[3] = 0.0
    group.set_joint_value_target(group_variable_values)

    plan3 = group.plan()
    group.execute(plan3)

def arm_right_position2():
    
    group = moveit_commander.MoveGroupCommander("arm")
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = -1.6
    group_variable_values[1] = 0.9
    group_variable_values[2] = -0.9
    group_variable_values[3] = 1.2
    group.set_joint_value_target(group_variable_values)

    plan3 = group.plan()
    group.execute(plan3)

def arm_right_position3():
    
    group = moveit_commander.MoveGroupCommander("arm")
    group_variable_values = group.get_current_joint_values()
    group_variable_values[0] = -1.6
    group_variable_values[1] = 1.2
    group_variable_values[2] = -0.9
    group_variable_values[3] = 1.2
    group.set_joint_value_target(group_variable_values)

    plan3 = group.plan()
    group.execute(plan3)

def gripper_open():
    gripper_msg.data = [-0.009]
    pub3.publish(gripper_msg)

def gripper_close():
    gripper_msg.data = [0.009]
    pub3.publish(gripper_msg)

def shutdown_flask():
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with Werkzeug')
    func()
