# uArm of WoT-SYS

This source realize controlling uArm(robot arm) by sending remote request, it contains the source code, launch file, guides etc.

## General
- Use ROS Kinetic and uArm-Python-SDK
- Use Postman to send remote requests
- Install ros serial package
- Clone this repo to get the code
- Download uarm python sdk in https://github.com/uArm-Developer/uArm-Python-SDK.git
- Install uArm-Python-SDK(some dependencies may be needed)
- Download Postman in https://www.getpostman.com/downloads/

## Workflow

Code have following functions: beep, beep with time, gohome, set home location, turnleft, turnright, go to a desired position with desired speed, grip object, drop object.

- Connect to a same LAN in computer and open postman
- Run roscore, remember to check ROS_MASTER_URI and ROS_IP in bashrc of raspberry pi
- Connect uArm to raspberry pi and get USB permission to access uArm, run `sudo chmod 666 /dev/ttyACM0`
- Change directory and Run `python main.py` 
- send remote requests through postman




