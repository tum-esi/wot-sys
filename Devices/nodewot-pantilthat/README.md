# Pan Tilt HAT

WoTified Pimoroni Pan Tilt HAT providing its basic functions as WoT property and actions over HTTP and MQTT

## Installation

- get the latest node: 
```bash
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install -y nodejs
```
- To build (transcompiling Typescript to javascript): `npm run build`
- To run the code: `npm run start` 


### API reference

Below is the API reference used to build this project

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



