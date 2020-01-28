# Sequence Diagrams

Each sequence diagram has an own mashup. You can find these in the mashup directory. Below you can see the same description you can find commented in the mashup itself.

## Mashups using EVENTS for triggering ACTIONS

### mashup_dobotmagician_basic_full

Description:

This mashup contains the following things:
    - two color sensors
    - two infrared sensors
    - two conveyor belts, moved by stepper motors
    - one dobot magician

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the robot moves this object to a color detection. Depending on the color, 
the robot moves the object to a specified position. After that the robot is ready for a new command from the same conveyor belt or for a command from the other conveyor belt. At the 
end of the moving operation, the conveyor belt restarts automatically again. 

### mashup_dobotmagician_detect&stop

Description:

This mashup contains the following things:
    - two infrared sensors
    - two conveyor belts, moved by stepper motors

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object.


### mashup_dobotmagician_detect&stop&color

Description:

This mashup contains the following things:
    - two color sensors
    - two infrared sensors
    - two conveyor belts, moved by stepper motors
    - one dobot magician

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the robot moves this object to a color detection. Regardless the color, 
the robot moves the object still to the same specified position. After that the robot is ready for a new command from the same conveyor belt or for a command from the other conveyor belt. 
At the end of the moving operation, the conveyor belt restarts automatically again. 

### mashup_dobotmagician_detect&stop&grip&drop

Description:

This mashup contains the following things:
    - two infrared sensors
    - two conveyor belts, moved by stepper motors
    - one dobot magician

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the robot moves the object still to the same specified position. 
After that the robot is ready for a new command from the same conveyor belt or for a command from the other conveyor belt. At the end of the moving operation, the conveyor belt 
restarts automatically again. 

### mashup_dobotmagician_detect&stop&movebelt

Description:

This mashup contains the following things:
    - two infrared sensors
    - two conveyor belts, moved by stepper motors

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the corresponding conveyor belt restarts after some time.

### mashup_dobotmagician_only1side

Description:

This mashup contains the following things:
    - one color sensors
    - one infrared sensors
    - one conveyor belts, moved by stepper motors
    - one dobot magician

The conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the robot moves this object to a color detection. Depending on the color, 
the robot moves the object to a specified position. After that the robot is ready for a new command from the conveyor belt. At the end of the moving operation, the conveyor belt 
restarts automatically again. 

## Mashups using SETINTERVAL for triggering the ACTIONS

### mashup_dobotmagician_polling

This is a mashup that uses the devices of the dobot magician setup and is based on SETINTERVAL for triggering the ACTIONS.

Description:

This mashup contains the following things:
    - two color sensors
    - two infrared sensors
    - two conveyor belts, moved by stepper motors
    - one dobot magician

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the robot moves this object to a color detection. Depending on the color, 
the robot moves the object to a specified position. After that, the robot is ready for a new command from the same conveyor belt or for a command from the other conveyor belt. At the 
end of the moving operation, the conveyor belt restarts automatically again. 

