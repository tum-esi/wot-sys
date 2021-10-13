
## Program Flow
* replace.py converts the thing model in TM.json to thing description and wites it into convertedTD.json.

    * Thing model in TM.json can be configured using config.json 

* ur10_flask.py is the main server script which reads the thing description from convertedTD.json

    * Some constant values in ur10_flask.py is read from constants.json

## What to Change

* You need to change the routerIP, routerPort, HTTP_IP_ADDRESS and robotIP in config.json according to your setup.

## Dependencies

* Python >=3.6
* ur_rtde
* Flask
* jsonschema
* Pillow

## Installation

- Install python: `sudo apt-get install python3 python3-pip`
- install requirements: `pip3 install -r requirement.txt`
- To run the program: `python3 ur10_flask.py`
- Make sure to change the config.json according to your setup.

