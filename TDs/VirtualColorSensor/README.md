# Virtual Color Sensor
## You can simulate one or more color sensors using the TD files and the appropriate prepared configuration files 

## Prerequisites
All systems require:
* [NodeJS](https://nodejs.org/) version 10+ (e.g., 10.13.0 LTS) 

### Linux
Meet the [node-gyp](https://github.com/nodejs/node-gyp#installation) requirements:
* Python 2.7 (v3.x.x is not supported)
* make
* A proper C/C++ compiler toolchain, like GCC

### Windows
Install the Windows build tools through a CMD shell as administrator:
```
npm install -g --production windows-build-tools
```

### Mac OS
Meet the [node-gyp](https://github.com/nodejs/node-gyp#installation) requirements:
```
xcode-select --install
```

## How to start a virtual thing
### Install this package
Clone this repository and go into it:
```
git clone https://github.com/tum-ei-esi/virtual-thing
cd virtual-thing
```
Install dependencies and build project:
```
npm install 
npm run build
``
### Start the virtual color sensor based on its TD and configuration file
you can create the described virtual color sensors based on its TD using the configuration files in the following way:
```
node -c virtual_color_sensor_http_localhost_8081.conf.json virtual_color_sensor_http_localhost_8081.json
```
node -c virtual_color_sensor_http_localhost_8082.conf.json virtual_color_sensor_http_localhost_8082.json
```
node -c virtual_color_sensor_mqtt.conf.json virtual_color_sensor_mqtt.json
```







