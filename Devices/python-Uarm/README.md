# uArm of WoT-SYS

This repository contains the code for controlling the Uarm (robot arm) over HTTP requests as well as MQTT commands.
It also includes a Thing Description detailing all possible interactions.

## General

- Create a folder called Uarm
- Clone this repo to your RaspberryPi (Code tested for RP3 and 4 with Rasbian Buster)
- Change static IP adresses of MQTT Broker and Flask Server in `main.py`
- Install an MQTT Broker (eg. Mosquitto) on the RP or use an already existing Broker in the Network
- Connect the Uarm robot over USB to the RaspberryPi
- Download Uarm python sdk in https://github.com/uArm-Developer/uArm-Python-SDK.git
- Install the Uarm python SDK (`sudo python3 setup.py install`)
- Install dependancies needed for the python code.
- `pip3 install jsonschema`
- `pip3 install paho-mqtt`
- After all dependancies are installed, execute `python3 main.py`


##Workflow

- Run `python3 main.py` (The Robot should beep once and after 20s the Flask Server should start running)
- Now one can interact with the Robot over HTTP requests or MQTT commands 
