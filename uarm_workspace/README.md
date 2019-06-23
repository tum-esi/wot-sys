# uArm of WoT-SYS

This source uses ROS to control uArm(robot arm), it contains the source code, launch file, guides etc.

## General
- Use ROS Kinetic and uArm-Python-SDK
- Install ros serial package
- Clone this repo to get the code
- Download uarm python sdk in https://github.com/uArm-Developer/uArm-Python-SDK.git
- Install uArm-Python-SDK(some dependencies may be needed)

## Gripper Template

The gripper template realize taking one object from one position to another position with gripper

- Change to your workspace
- Connect uArm to computer and get USB permission to access uArm, run `sudo chmod 666 /dev/ttyACM0`
- Run `roslaunch swiftpro demo.launch` 

## Pump Template

The pump template realize taking one paper from one position to another position with pump

- Change to your workspace
- Connect uArm to computer and get USB permission to access uArm, run `sudo chmod 666 /dev/ttyACM0`
- Run `roslaunch swiftpro pump.launch`


