/*
This Mashup involves devices with "id":"de:tum:ei:esi:mearmpi:192.168.0.104" and "id":"de:tum:ei:esi:sensehat:192.168.0.106".

This Mashup closes the grip of the robot if the button on Sensehat is pressed
*/
const Robot_TD_ADDRESS = "http://192.168.0.104:8080/MeArmPi";
const senseHAT_TD_ADDRESS = "http://192.168.0.106:8080/SenseHat";

WoT.fetch(Robot_TD_ADDRESS).then(async (robotTD) => {
    
    WoT.fetch(senseHAT_TD_ADDRESS).then(async (senseHatTD) => {
        
        robotThing = WoT.consume(robotTD);
        senseHatThing = WoT.consume(senseHatTD);
        senseHatThing.events.joystickPress.subscribe(x => {
            robotThing.actions["dance"].invoke();
            e => {
                console.log("onError: %s", e)
            },
            () => {
                console.log("onCompleted");
            }
        }).catch((err) => console.log(err))
    });
});
