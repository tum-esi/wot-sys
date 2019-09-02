
# Motor Controller (Pololu TB9051FTG Dual)

## General

Code to run motors using Pololu TB9051FTG Dual and Raspberry Pi

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
- To run the program: `npm start` or  `sudo node index.js` (sudo is needed to control pins of Paspberry Pi)