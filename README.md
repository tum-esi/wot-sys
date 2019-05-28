# WoT-SYS

This is a project that contains the source code, description, guides etc. of the WoT System that will be used by students of ESI.

## General
- Clone this repo to get the code
- All the devices are hard coded to connect the a TD Directory at 192.168.0.100. IF using another TD Directory please change the code accordingly. 
- Once all devices are running, you can access all the available TD by visiting the TD directory at `192.168.0.100:8080`or `TD-Directory.local:8080`

## Postman

The collections with all interactions are found under Postman and also [here](https://www.getpostman.com/collections/ad2fa98aaf79b7d1a448)

The collection documentation is available [here](https://documenter.getpostman.com/view/2152549/S17wNS8o)

## Mashup

Example mashups and clients for students are found under Mashups

# ESP Light Sensor

## Requirements

self contained - runs on micropython

## Installation
- get the micropython firmware from http://micropython.org/download#esp8266
- run `pip install esptool`
- run `esptool.py --port <dev location, e.g. /dev/ttyUSB0> erase_flash`
- run `esptool.py --port <dev location> --baud 460800 write_flash --flash_size=detect 0 <name of the firmware you downloaded>`
- upload the `main.py` script to ESP8266 with `ampy` following the instructions from https://learn.adafruit.com/micropython-basics-load-files-and-run-code/install-ampy
- to use the REPL of the ESP8266, connect it to your USB port and use a serial communication program. Example under Linux: `picocom <dev location> -b 115200`
- to find out the MAC address of the ESP, follow the instructions on https://forum.micropython.org/viewtopic.php?t=1890
- assign a static IP adress to this MAC on the router's configuration page
- You're set:)

# PiCamera

## Network Configuration

WiFi SSID and password are stored at: /etc/wpa_supplicant/wpa_supplicant.conf

## Requirements
- python >= 3.5
- picamera
- flask

## Installation
- Install python: `sudo apt-get install python3 python3-pip`
- install requirements: `pip3 install -r requirement.txt`
- To run the program: `python3 camera_server.py`
- Make sure that this script runs automatically on startup. Crontab can do this on Linux.
- The server runs on port 8080

# DotStar

## Requirements
- flask
- RPI.GPIO
- adafruit-blinka
- adafruit-circuitpython-dotstar
- You have to enable SPI on the board.

## Installation
- Install python: `sudo apt-get install python3 python3-pip`
- Install requirements: `pip3 install -r requirement.txt`
- Enable SPI on the Raspberry Pi. You can use this in the GUI or the CLI config interface: `raspi-config`
- To run the program: `python3 dotstar.py`
- Make sure that this script runs automatically on startup. Crontab can do this on Linux.
- The server runs on port 8080

# MeArmPi

## Requirements
- NodeJS 8 or newer
- You have to enable SPI on the board.

## Installation
- get the latest node: 
```bash
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install -y nodejs
```
- `cd` inside the wot-mearmpi folder and run: `npm install`
- Build: `npm run build`
- To run the program: `node index.js`
- Make sure that this script runs automatically on startup. Crontab can do this on Linux.
- The server runs on port 8080

# SenseHat

## Requirements
- NodeJS >= 8.11.2 (version between 8.10.0 and 8.11.1 have a bug that breaks this code)
- npm >= 6.5

## Installation
- get the latest node: 
```bash
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install -y nodejs
```
- `cd` inside the wot-sensehat folder and run: `npm install`
- Build: `npm run build`
- To run the program: `node index.js`
- Make sure that this script runs automatically on startup. Crontab can do this on Linux.
- The server runs on port 8080