# WoT-SYS

This is a project that contains the source code, description, guides etc. of the WoT System that will be used by students of ESI.

## General
- Clone this repo to get the code
- All the devices are hard coded to connect the a TD Directory at 192.168.0.100. IF using another TD Directory please change the code accordingly. 
- Once all devices are running, you can access all the available TD by visiting the TD directory at `192.168.0.100:8080`or `TD-Directory.local:8080`

## Postman

The collections with all interactions are found under Postman and also [here](https://www.getpostman.com/collections/ad2fa98aaf79b7d1a448)

The collection documentation is available [here](https://documenter.getpostman.com/view/2152549/S17wNS8o)

## Hostnames

Blue Mearm: TUMEIESI-MeArmPi-Blue
Orange Mearm: TUMEIESI-MeArmPi-Orange
Normal RPi Camera: TUMEIESI-Rpi-Camera
IR RPi Camera: TUMEIESI-Rpi-Camera-IR
Dotstar: TUMEIESI-DotStar
Sensehat-106 : TUMEIESI-SenseHat-106
Sensehat-107 : TUMEIESI-SenseHat-107
Hue-related devices: Philips-hue

## Mashup

Example mashups and clients for students are found under Mashups

# ESP Light Sensor

## Requirements

self contained - runs on micropython

## What to Change

* This connects to a WiFi with SSID and PW specified in the beginning of `main.py`. You should change them according to your WiFi setup
* The provided TD will be updated in a new network since the IP address of the ESP will change

## Installation
- You will flash the program via the serial port. Make sure that you have read/write rights to the port by following [this guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/establish-serial-connection.html).
- Get the micropython firmware from [here](http://micropython.org/download#esp8266) or use version provided in at esp8266 folder.
- Run `pip install esptool` to install the tool that allows you to flash programs to the ESP board
- Run `esptool.py --port <dev location, e.g. /dev/ttyUSB0> erase_flash` to remove the previous program or firmware
- Run `esptool.py --port <dev location> --baud 460800 write_flash --flash_size=detect -fm dio 0 <name of the firmware>` to download the firmware into ESP board. This is not your program, just the firmware.
- Upload the `main.py` script to ESP8266 with `ampy` following the instructions from [Adafruit](https://learn.adafruit.com/micropython-basics-load-files-and-run-code/install-ampy)
  - Install ampy: `pip install adafruit-ampy`
  - Check that it is installed with `ampy --help`
  - You can run the code with `ampy --port /dev/ttyUSB0  run main.py`
  - You can upload it as the main program via `ampy --port /dev/ttyUSB0 put main.py /main.py`
- to use the REPL of the ESP8266, connect it to your USB port and use a serial communication program. Example under Linux: `picocom <dev location> -b115200`
  - You will need to press enter to enter into the console.
  - To exit, press CTRL+A+X
- to find out the MAC address of the ESP, follow the instructions on https://forum.micropython.org/viewtopic.php?t=1890
- assign a static IP adress to this MAC on the router's configuration page
  - You may not see it in the DHCP clients list
- When you see the LED lit, it means that it has connected to WiFi

# PiCamera

## Network Configuration

* WiFi SSID and password are stored at: /etc/wpa_supplicant/wpa_supplicant.conf
* You can also launch the desktop and change to another WiFi

## Requirements
- python >= 3.5
- picamera
- flask

## Installation
- Install python: `sudo apt-get install python3 python3-pip`
- install requirements: `pip3 install -r requirement.txt`
- To run the program: `python3 camera_server.py`
- Make sure that this script runs automatically on startup. Crontab can do this on Linux. Use `crontab -e` to configure crontab. Actually launch.sh does the launch
- The server runs on port 8080

## Warning

Keyboard Layout is American

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

## Warning

Keyboard Layout is German

# MeArmPi

## Network Configuration

* WiFi SSID and password are stored at: /etc/wpa_supplicant/wpa_supplicant.conf
* You can also launch the desktop and change to another WiFi

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
- Make sure that this script runs automatically on startup. Crontab can do this on Linux. Run `sudo crontab -e` to get its config
- The server runs on port 8080


## Warning

Keyboard Layout is German


# Philips HUE Bridge

This bridge has been configured using the white light starter pack that contains 3 white lights and a switch. 

### Get it running

* Follow the Philips HUE in-app guide to get your bridge started.
* Follow the [developer page](https://developers.meethue.com/develop/get-started-2/) to get an API Key
* The base URI is using our API Key, replace it with yours:
  * Look for the base URI in the TD
  * Replace the part after `/api/` with your API key.
  * This TD uses `Philips-hue` as hostname, in case you have multiple HUE Bridges, you will need to change this in the base URI as well

# Philips HUE Lights

This is a TD of a light bulb that connects to a Philips HUE Bridge.

### Get it running

* Follow the Philips HUE in-app guide to get your bridge started.
* Follow the [developer page](https://developers.meethue.com/develop/get-started-2/) to get an API Key
* The base URI is using our API Key and Light number, replace them with yours:
  * Look for the base URI in the TD
  * Replace the part between `/api/` and `/lights/` with your API key.
  * This TD is representing light number 1. You should replace the part after `/lights/` with your light's number.
  * This TD uses `Philips-hue` as hostname, in case you have multiple HUE Bridges, you will need to change this in the base URI as well

# Philips HUE Dimmer Switch

This is a TD of a dimmer switch that connects to a Philips HUE Bridge and that can be used to turn on and off different HUE Lights.

### Get it running

* Follow the Philips HUE in-app guide to get your bridge started.
* Follow the [developer page](https://developers.meethue.com/develop/get-started-2/) to get an API Key
* The base URI is using our API Key and Sensor number, replace them with yours:
  * Look for the base URI in the TD
  * Replace the part between `/api/` and `/sensors/` with your API key.
  * This TD is representing the sensor number 2. You should replace the part after `/sensors/` with your sensor's number.
  * This TD uses `Philips-hue` as hostname, in case you have multiple HUE Bridges, you will need to change this in the base URI as well

# Philips HUE Software-based Daylight Sensor

This is a TD of a daylight sensor that runs on a Philips HUE Bridge and that can be used to learn whether there is daylight in the Bridge's location or not. It is based on the GPS location.

### Get it running

* Follow the Philips HUE in-app guide to get your bridge started.
* Follow the [developer page](https://developers.meethue.com/develop/get-started-2/) to get an API Key
* The base URI is using our API Key and Sensor number, replace them with yours:
  * Look for the base URI in the TD
  * Replace the part between `/api/` and `/sensors/` with your API key.
  * This TD is representing the sensor number 1. You should replace the part after `/sensors/` with your sensor's number.
  * This TD uses `Philips-hue` as hostname, in case you have multiple HUE Bridges, you will need to change this in the base URI as well

# Motor Controller (Pololu TB9051FTG Dual)

### Pinout

* Left Motor negative pole -> M1A
* Left Motor positive pole -> M1B

* Right Motor negative pole -> M2B
* Right Motor positive pole -> M2A

* Battery Positive -> VIN
* Battery GND ->GND 

* No need to make any changes between controller hat and raspberry pi, pins may vary between other brands and controllers which can be changed in motorDriver.js

## Installation

- get the latest node: 
```bash
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install -y nodejs
```
- `cd` inside the motor-driver folder and run: `npm install`
- To run the program: `npm start`
- `npm run debug` can be used to control motors via wasdx or arrow keys
- `npm run debug-only` for disabling Wot and only using keyboard input while testing or debugging
