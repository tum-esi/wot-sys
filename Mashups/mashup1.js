const Robot_TD_ADDRESS = "http://192.168.0.105:8080/MeArmPi";

const LightSensor_TD_ADDRESS = "http://192.168.0.102/";

const senseHAT_TD_ADDRESS = "http://192.168.0.106:8080/SenseHat";

const strip_TD_ADDRESS = "http://192.168.0.103:8080/";

WoT.fetch(Robot_TD_ADDRESS).then(async (robotTD) => {

    WoT.fetch(senseHAT_TD_ADDRESS).then(async (senseHatTD) => {

        WoT.fetch(LightSensor_TD_ADDRESS).then(async (lightSensorTD) => {

            WoT.fetch(strip_TD_ADDRESS).then(async (stripTD) => {

                robotThing = WoT.consume(robotTD);
                lightSensorThing = WoT.consume(lightSensorTD);
                senseHatThing = WoT.consume(senseHatTD);
                stripThing = WoT.consume(stripTD);
                
                stripThing.actions["shutdown"].invoke();
                senseHatThing.events.joystickPress.subscribe(x=>{
                    console.log("pressed");
                    stripThing.actions["random"].invoke();
                },
                    e => {
                        console.log("onError: %s", e)
                },
                    () => {
                        console.log("onCompleted");
                    }
                    )

                setInterval(async () => {
                    var intensity = await lightSensorThing.properties.intensity.read();
                    //    console.log(intensity);

                    if (intensity < 200) {
                        robotThing.actions["closeGrip"].invoke();
                        robotThing.actions["moveBaseTo"].invoke(75);
                        senseHatThing.actions["flashMessage"].invoke({
                            "textString": "<"
                        });
                    } else {
                        robotThing.actions["openGrip"].invoke();
                        robotThing.actions["moveBaseTo"].invoke(-75);
                        senseHatThing.actions["flashMessage"].invoke({
                            "textString": ">"
                        });
                    }
                }, 1000);


            });

        });

    });

})