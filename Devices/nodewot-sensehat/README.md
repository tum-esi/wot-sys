# SenseHat

## Network Configuration

* WiFi SSID and password are stored at: /etc/wpa_supplicant/wpa_supplicant.conf
* You can also launch the desktop and change to another WiFi

## Requirements
- NodeJS >= 8.11.2 (version between 8.10.0 and 8.11.1 have a bug that breaks this code)
- npm >= 6.5

## Installation
- get the latest node: 
```bash
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt-get install -y nodejs
```
- `cd` inside the wot-sensehat folder and run: `npm install`
- Build: `npm run build`
- To run the program: `node index.js`
- Make sure that this script runs automatically on startup. Crontab can do this on Linux. Run `sudo crontab -e` to get its config
- The server runs on port 8080
