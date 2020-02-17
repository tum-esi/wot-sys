/*
This is a mashup that uses the devices of the dobot magician setup and is based on EVENTS for triggering ACTIONS.

Description:

This mashup contains the following things:
    - one color sensors
    - one infrared sensors
    - one conveyor belts, moved by stepper motors
    - one dobot magician

The conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the robot moves this object to a color detection. Depending on the color, 
the robot moves the object to a specified position. After that the robot is ready for a new command from the conveyor belt. At the end of the moving operation, the conveyor belt 
restarts automatically again. 

*/

const Robot_TD_ADDRESS = "http://192.168.0.127:8080/DobotMagician";
const Infrared1_TD_ADDRESS = "http://192.168.0.128:8080/InfraredSensor1";
const ConveyorBelt1_TD_ADDRESS = "http://192.168.0.130:8080/StepperMotor";
const VirtualColorSensor1_TD_ADDRESS = "http://localhost:8081/VirtualColorSensor1";

WoTHelpers.fetch(Robot_TD_ADDRESS).then(async (robotTD) => {
    let robotThing = await WoT.consume(robotTD);
    console.info("=== TD ===");
    console.info(robotTD);
    console.info("==========");

    // Move dobot magician to default start position
    await robotThing.invokeAction("startPosition");

    WoTHelpers.fetch(ConveyorBelt1_TD_ADDRESS).then(async (conveyorbelt1TD) => {
        let conveyorThing1 = await WoT.consume(conveyorbelt1TD);

        WoTHelpers.fetch(VirtualColorSensor1_TD_ADDRESS).then(async (color1TD) => {
            let colorThing1 = await WoT.consume(color1TD);

            WoTHelpers.fetch(Infrared1_TD_ADDRESS).then(async (infrared1TD) => {
                let infraredThing1 = await WoT.consume(infrared1TD);

                // Start moving conveyor belt 1
                conveyorThing1.invokeAction("startBeltForward");
                // In case of an event of the infrared sensor, call function detectedObjectPosition1
                infraredThing1.subscribeEvent("detectedObject",
                x => detectedObjectPosition1(),
                e => console.error("Error: %s", e),
                () => console.info("Completed")
                );

                async function detectedObjectPosition1(){
                    // Stop the conveyor belt
                    await conveyorThing1.invokeAction("stopBelt");
                    // The robot picks the object and moves it to the color sensor
                    await robotThing.invokeAction("pickObjectPosition1");
                    await robotThing.invokeAction("moveToColorSensor1");
                    // Read the color sensor and call the function colorDetectedPosition1
                    myColorOutput1 = await colorThing1.invokeAction("readColor");
                    await colorDetectedPosition1(myColorOutput1)
                    // Restart the conveyor belt
                    await conveyorThing1.invokeAction("startBeltForward");
                }

                async function colorDetectedPosition1(x){
                    // Depending on color, the robot moves the object at the right position
                    switch (x) {
                        case "Red":
                            await robotThing.invokeAction("moveObjectRed");
                            break;
                        case "Green":
                            await robotThing.invokeAction("moveObjectGreen");
                            break;
                        case "Blue":
                            await robotThing.invokeAction("moveObjectBlue");
                            break;
                        case"None":
                            await robotThing.invokeAction("moveObjectNone");
                            break;
                    }
                }    
            }).catch( ()=>{
                console.log("Could not get the InfraredSensor1 TD")
            });
        }).catch( ()=>{
            console.log("Could not get the ColorSensor1 TD")
        });
    }).catch( ()=>{
        console.log("Could not get the ConveyorBelt1 TD")
    });   
         
}).catch( ()=>{
    console.log("Could not get the Robot TD")
});
