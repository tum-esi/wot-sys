# BaseWoT
This is a project for making the proccess of creating a WoT enabled device easier by giving base functions and code and putting fill in comments.

### Get it running
*   #### Change name package.json file
  Change name
  Change description
  Add needed dependencies
*   #### Go to src/base.ts and fill needed places
  fill in empty quotation marks in produce function
  fill in addProperties and addActions if needed
*   #### If Coap or MQTT is needed 
  add dependency @node-wot/binding-xxx (e.g. binding-http, binding-mqtt)
  uncomment related lines in index.js

*   #### Follow installation steps 

## Installation

- get the latest node: 
```bash
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install -y nodejs
```
- To convert ts to js: `npm run build`
- To install packages and convert ts to js  `npm run buildAll`
- `npm run start` to run the code


### API reference

This library provides the following APIs
1) Pimoroni API
The library impements
   pan(angle)        - pan angle is -90..+90
   servo_one(angle)  - pan angle is -90..+90
   tilt(angle)       - tilt angle is -80..+80
   servo_two(angle)  - tilt angle is -80..+80
   goto_home()       - Set Pan to 0 and Tilt to 0
   light_type()      - Set light type to RGB, GRB, RGBW or GRBW
   light_mode()      - Set light mode to PWM or WS2812
   brightness()      - Set brightness in PWM mode between 0 and 255
   set_all()         - Set colour of all pixels in WS2812 mode
   set_pixel()       - Set colour of individual pixel in WS2812 mode
   show()            - Make changes to pixel settings take effect

2) Continual Move API
The API is based on analogue CCTV systems where a command will start the pan
or tilt motors moving and the motors will continue to move until a stop() command
is issued.
The API commands are
  pan_left(speed)  - speed range is 0..15
  pan_right(speed) - speed range is 0..15
  tilt_up(speed)   - speed range is 0..15
  tilt_down(speed) - speed range is 0..15
  stop()
The library updates the pan and tilt position 10 times per second

3) An Exit API
  close() - used to close the spawned Python worker process and to stop an internal timer to allow NodeJS to terminate.



