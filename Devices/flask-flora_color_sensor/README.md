## Requirements

* Pyhton 3.7
* GPIO Zero (should be installed by default)
* Pip3 9.0.1
* adafruit-circuitpython-tcs34725

## Installation
1. Clone the repository in a local file
2. Enable I2C interface of your Raspberry Pi
3. Go into the that directory in terminal and run `pip3 install -r requirements.txt`.
4. Run `pyhton3 flora.py`

## Important Notes

1. In flora.py, replace TD_DIRECTORY_ADDRESS, if you do not have a directory comment out `submit_td()`.
2. In flora.py, replace the router address you connect.
3. You can get the Thing Description from `localhost:port/`.
4. You can use other ports as well.

 


