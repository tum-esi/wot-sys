Methods
---

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