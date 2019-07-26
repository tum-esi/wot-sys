/*
This Mashup closes the grip of the robot if the button on Sensehat is pressed
*/
const Robot_TD_ADDRESS = "http://TUMEIESI-MeArmPi-Orange.local:8080/MeArmPi";
const senseHAT_TD_ADDRESS = "http://TUMEIESI-SenseHat-107.local:8080/SenseHat/";

WoT.fetch(Robot_TD_ADDRESS).then(async (robotTD) => {

    WoT.fetch(senseHAT_TD_ADDRESS).then(async (senseHatTD) => {

                        robotThing = WoT.consume(robotTD);
                        senseHatThing = WoT.consume(senseHatTD);
                            setInterval(async () => {
                                senseHatThing.events.joystickPress.subscribe(x => {
                                    robotThing.actions["closeGrip"].invoke();
                                    e => {
                                        console.log("onError: %s", e)
                                    },
                                    () => {
                                        console.log("onCompleted");
                                    }
                    }).catch((err) => console.log(err))
                },30000).catch((err) => console.log(err))
            });
        });
