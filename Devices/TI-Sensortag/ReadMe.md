## README

### Requirements
Nodejs version:+10, npm

### Installation Instructions

* Install nodejs and npm.  
```bash
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
sudo apt-get install -y nodejs
```
* Go to directory of TI-Sensortag in terminal
* Run `npm install tsc` 
* Run `npm install`
* Run `npm start`

### Warnings

* You need to change the bluetooth addresses in conf.json file with the bluetooth addresses of your TI-Sensortags.
* If you want to enable acceleration, gyroscope, magnetometer sensors you need to change the "gyro_acc_mag" value in conf.json file to true but sensor values are not correct at the moment.