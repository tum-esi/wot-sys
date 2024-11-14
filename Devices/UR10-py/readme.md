
## Program Flow

* replace.py converts the thing model in TM.json to thing description and wites it into convertedTD.json.

  * Thing model in TM.json can be configured using config.json

* ur10_flask.py is the main server script which reads the thing description from convertedTD.json

  * Some constant values in ur10_flask.py is read from constants.json

## What to Change

* You need to change the routerIP, routerPort, HTTP_IP_ADDRESS and robotIP in config.json according to your setup.

## Dependencies

* Python >= 3.10
* ur_rtde
* Flask
* jsonschema
* pillow

## Installation

* Install python: `sudo apt-get install python3 python3-pip`
* install requirements: `pip3 install -r requirement.txt`
* To run the program: `python3 ur10_flask.py`
* Make sure to change the config.json according to your setup.

## UR-RTDE

The script uses UR-RTDE to control the robot in real-time. However, it does not by default expose the Onrobot gripper attached to the UR10. To do this, a custom script has to be loaded to the robot (via USB). The script is split in 2 parts [rtde_control.script](rtde_control.script) and [rtde_init.script](rtde_init.script). Add [rtde_control.script](rtde_control.script) as the main script and [rtde_init.script](rtde_init.script) as the `BeforeSequence` script. Then, when the python script connects to the UR10, you need to stop the default running script and load the custom script.
