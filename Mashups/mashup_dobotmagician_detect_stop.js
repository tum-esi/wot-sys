/*
This is a mashup that uses the devices of the dobot magician setup and is based on EVENTS for triggering ACTIONS.

Description:

This mashup contains the following things:
    - two infrared sensors
    - two conveyor belts, moved by stepper motors

Each conveyor belt starts at the beginning, and stops when the infrared sensor detects an object.

*/

const Infrared1_TD_ADDRESS = "http://192.168.0.128:8080/InfraredSensor1";
const Infrared2_TD_ADDRESS = "http://192.168.0.129:8080/InfraredSensor2";
const ConveyorBelt1_TD_ADDRESS = "http://192.168.0.130:8080/StepperMotor";
const ConveyorBelt2_TD_ADDRESS = "http://192.168.0.131:8080/StepperMotor";


    WoTHelpers.fetch(ConveyorBelt1_TD_ADDRESS).then(async (conveyorbelt1TD) => {
        let conveyorThing1 = await WoT.consume(conveyorbelt1TD);

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
                }   

            }).catch( ()=>{
                console.log("Could not get the ConveyorBelt1 TD")
            });
        }).catch( ()=>{
            console.log("Could not get the ColorSensor1 TD")
        });   

    WoTHelpers.fetch(ConveyorBelt2_TD_ADDRESS).then(async (conveyorbelt2TD) => {
        let conveyorThing2 = await WoT.consume(conveyorbelt2TD);

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
                }

            }).catch( ()=>{
                console.log("Could not get the ConveyorBelt2 TD")
            });
        }).catch( ()=>{
            console.log("Could not get the ColorSensor2 TD")
        });         
