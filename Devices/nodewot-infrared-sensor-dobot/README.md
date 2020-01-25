# E18-D80NK Infrared Sensor

![Infrared Sensor](https://www.microchip.lk/wp-content/uploads/2018/03/ir-barrier-sensor-e18-d80nk.jpg)


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