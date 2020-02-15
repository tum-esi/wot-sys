# E18-D80NK Infrared Sensor

![Infrared Sensor](https://www.microchip.lk/wp-content/uploads/2018/03/ir-barrier-sensor-e18-d80nk.jpg)

### Raspberry Pi Configuration

You can find more information about the following steps here:  
* [![onoff](https://www.npmjs.com/package/onoff)](https://www.npmjs.com/package/onoff)   

```
1)  npm install
2)  (npm install onoff) -> already included in package.json
3)  npm run build
4)  npm run start
```
In case of some problems with npm run start, try:
```
sudo shutdown -r 0 
```
and wait until the rpi is ready. 

### Autostart execution Raspberry Pi

Use the following terminal command:
```
crontab -e
```
Write the commands that need to be executed at the reboot of the Raspberry Pi.
Example text:

```
@reboot sleep 10 && ~/Desktop/FolderOfTheThingProgram && npm run start
```
Save and close.

### Schematics

![wiring](Devices/nodewot-infrared-sensor-dobot/Schematics/Schematics_Infrared_Sensor.png)