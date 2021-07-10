# Unicorn pHAT

## Installation

- Enable SPI and i2c in your RPi Configuration
- Install Node.js `curl -sL https://deb.nodesource.com/setup_10.x | sudo bash -`
- `npm install`
- `npm install -g typescript` for tsc
- `npm run build`
- Modify the TD directory address if you have one in index.js

## Running
- `sudo npm start` sudo is needed to access `/dev/mem`
