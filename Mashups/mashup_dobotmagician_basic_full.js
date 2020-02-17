/*
This is a mashup that uses the devices of the dobot magician setup and is based on EVENTS for triggering ACTIONS.

Description:

This mashup contains the following things:
    - two color sensors
    - two infrared sensors
    - two conveyor belts, moved by stepper motors
    - one dobot magician

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object. After that, the robot moves this object to a color detection. Depending on the color, 
the robot moves the object to a specified position. After that the robot is ready for a new command from the same conveyor belt or for a command from the other conveyor belt. At the 
end of the moving operation, the conveyor belt restarts automatically again. 

*/

const Robot_TD_ADDRESS = "http://192.168.0.127:8080/DobotMagician";
const Infrared1_TD_ADDRESS = "http://192.168.0.128:8080/InfraredSensor1";
const Infrared2_TD_ADDRESS = "http://192.168.0.129:8080/InfraredSensor2";
const ConveyorBelt1_TD_ADDRESS = "http://192.168.0.130:8080/StepperMotor";
const ConveyorBelt2_TD_ADDRESS = "http://192.168.0.131:8080/StepperMotor";
const VirtualColorSensor1_TD_ADDRESS = "http://localhost:8081/VirtualColorSensor1";
const VirtualColorSensor2_TD_ADDRESS = "http://localhost:8082/VirtualColorSensor2";
// Variable robot busy 
var robotBusy = false;

WoTHelpers.fetch(Robot_TD_ADDRESS).then(async (robotTD) => {
    let robotThing = await WoT.consume(robotTD);
    console.info("=== TD ===");
    console.info(robotTD);
    console.info("==========");

    // Move dobot magician to default start position
    await robotThing.invokeAction("startPosition");

    async function resolveIfRobotNotBusy1(){
        return new Promise(resolve => { 
            var setIntervalID1 = setInterval(() => {
                if (!robotBusy){
                    setRobotBusy();
                    clearInterval(setIntervalID1);
                    resolve();
                } 
            }, 250);     // repeat every 250 ms
        });
    }

    async function resolveIfRobotNotBusy2(){
        return new Promise(resolve => {
            var setIntervalID2 = setInterval(() => {
                if (!robotBusy){
                    setRobotBusy();
                    clearInterval(setIntervalID2);
                    resolve();
                } 
            }, 250);     // repeat every 250 ms
        });
    }

    async function setRobotBusy(){
        robotBusy = true;    
    }

    async function setRobotNotBusy(){
        robotBusy = false; 
    }

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
                    // Wait until the robot has finished all tasks
                    await resolveIfRobotNotBusy1();
                    // The robot picks the object and moves it to the color sensor
                    await robotThing.invokeAction("pickObjectPosition1");
                    await robotThing.invokeAction("moveToColorSensor1");
                    // Read the color sensor and call the function colorDetectedPosition1
                    myColorOutput1 = await colorThing1.invokeAction("readColor");
                    await colorDetectedPosition1(myColorOutput1)
                    await setRobotNotBusy();
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

    WoTHelpers.fetch(ConveyorBelt2_TD_ADDRESS).then(async (conveyorbelt2TD) => {
        let conveyorThing2 = await WoT.consume(conveyorbelt2TD);

        WoTHelpers.fetch(VirtualColorSensor2_TD_ADDRESS).then(async (color2TD) => {
            let colorThing2 = await WoT.consume(color2TD);

            WoTHelpers.fetch(Infrared2_TD_ADDRESS).then(async (infrared2TD) => {
                let infraredThing2 = await WoT.consume(infrared2TD);

               // Start moving conveyor belt 2
               conveyorThing2.invokeAction("startBeltForward");
               // In case of an event of the infrared sensor, call function detectedObjectPosition2
               infraredThing2.subscribeEvent("detectedObject",
               x => detectedObjectPosition2(),
               e => console.error("Error: %s", e),
               () => console.info("Completed")
               );

               async function detectedObjectPosition2(){
                    // Stop the conveyor belt
                    await conveyorThing2.invokeAction("stopBelt");
                    // Wait until the robot has finished all tasks
                    await resolveIfRobotNotBusy2();
                    // The robot picks the object and moves it to the color sensor
                    await robotThing.invokeAction("pickObjectPosition2");
                    await robotThing.invokeAction("moveToColorSensor2");
                    // Read the color sensor and call the function colorDetectedPosition2
                    myColorOutput2 = await colorThing2.invokeAction("readColor");
                    await colorDetectedPosition2(myColorOutput2)
                    await setRobotNotBusy();
                    // Restart the conveyor belt
                    await conveyorThing2.invokeAction("startBeltForward");
                }

                async function colorDetectedPosition2(x){
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
                console.log("Could not get the InfraredSensor2 TD")
            });
        }).catch( ()=>{
            console.log("Could not get the ColorSensor2 TD")
        });
    }).catch( ()=>{
        console.log("Could not get the ConveyorBelt2 TD")
    });         
}).catch( ()=>{
    console.log("Could not get the Robot TD")
});
