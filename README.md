# WoT-SYS

This is a project that contains the source code, description, guides etc. of the WoT System that will be used by students of ESI.

# Instructions
- clone this repo
- `python3 td.py &`
- `python3 camera_server.py &`
- The server serving TD is listening to port `3000`; Whilst the server communicating with the camera is listening to port `5000`

# Requirements for Camera 
- python >= 3.5
- picamera
- flask

# Requirements for ESP Ligth Sensor 

Self contained

# ESP8266 light sensor instructions
- get the micropython firmware from http://micropython.org/download#esp8266
- run `pip install esptool`
- run `esptool.py --port <dev location, e.g. /dev/ttyUSB0> erase_flash`
- run `esptool.py --port <dev location> --baud 460800 write_flash --flash_size=detect 0 <name of the firmware you downloaded>`
- change `esp_main.py` to `main.py`
- upload the script to ESP8266 with `ampy` following the instructions from https://learn.adafruit.com/micropython-basics-load-files-and-run-code/install-ampy
- to use the REPL of the ESP8266, use the command `picocom <dev location> -b 115200`
- to find out the MAC address of the ESP, follow the instructions on https://forum.micropython.org/viewtopic.php?t=1890
- assign a static IP adress to this MAC on the router's configuration page
- You're set:)
