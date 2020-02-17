# Dobot Magician

![DobotMagician](Devices/nodewot-dobot-magician/Images/Dobot_Magician.png)

## Exposed Thing implementation based on: Exposed Thing with node-wot as Dependency
![Exposed Thing with node-wot as Dependency](https://github.com/eclipse/thingweb.node-wot/tree/master/examples/templates/exposed-thing)

### Raspberry Pi Configuration

You can find more information about the following step "pip install pydobot" after the Raspberry Pi Configuration at Python library for Dobot Magician with sliding rail.

```
1)  npm install
2)  pip install pydobot 
3)  npm run build
4)  npm run start
```
In case of some problems with npm run start, try:
```
sudo shutdown -r 0 
```
and wait until the rpi is ready. 

### Note
All the movement actions are synchronous, which means thath the request comes to the robot, the robot completes the movement and then responds.

Python library for Dobot Magician with sliding rail
===

You can find more information about the original files here:
[![CircleCI](https://circleci.com/gh/luismesas/pydobot)](https://circleci.com/gh/luismesas/pydobot)

Based on Communication Protocol V1.1.4 (_latest version [here](https://www.dobot.cc/downloadcenter.html?sub_cat=72#sub-download)_)


Installation of the pydobot library:
---

```
pip install pydobot
```

Then replace the following file given here in the directory lib -> pydobot.
```
dobot.py
```
This file contains also the commands for the sliding rail of the dobot magician. 


Methods
---

* **list_ports.comports()[0].device** Chooses the port used for the dobot, which is in this case the USB port.
* **Dobot(port, verbose=False)** Creates an instance of dobot connected to given serial port.
    * **port**: _string_ with name of serial port to connect
    * **verbose**: _bool_ will print to console all serial comms  
* **.pose()** Returns the current pose of dobot, as a tuple (x, y, z, r, j1, j2, j3, j4)
    * **x**: _float_ current x cartesian coordinate 
    * **y**: _float_ current y cartesian coordinate
    * **z**: _float_ current z cartesian coordinate
    * **r**: _float_ current effector rotation 
    * **j1**: _float_ current joint 1 angle 
    * **j2**: _float_ current joint 2 angle 
    * **j3**: _float_ current joint 3 angle 
    * **j4**: _float_ current joint 4 angle   
* **.pose_l()** Returns the current pose of sliding rail, (l)
    * **l**: _float_ current l pose of the sliding rail  
* **.move_to(x, y, z, r, wait=False)** queues a translation in dobot to given coordinates
    * **x**: _float_ x cartesian coordinate to move 
    * **y**: _float_ y cartesian coordinate to move 
    * **z**: _float_ z cartesian coordinate to move 
    * **r**: _float_ r effector rotation 
    * **wait**: _bool_ waits until command has been executed to return to process  
* **.move_to_l(x, y, z, r, l, wait=False)** queues a translation in dobot to given coordinates and for the sliding rail
    * **x**: _float_ x cartesian coordinate to move 
    * **y**: _float_ y cartesian coordinate to move 
    * **z**: _float_ z cartesian coordinate to move 
    * **r**: _float_ r effector rotation 
    * **l**: _float_ l coordinate of the sliding rail to move
    * **wait**: _bool_ waits until command has been executed to return to process  
* **.speed(velocity, acceleration)** changes velocity and acceleration at which the dobot moves to future coordinates
    * **velocity**: _float_ desired translation velocity 
    * **acceleration**: _float_ desired translation acceleration   
* **.suck(enable)**
    * **enable**: _bool_ enables/disables suction  
* **.grip(enable)**
    * **enable**: _bool_ enables/disables gripper  


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