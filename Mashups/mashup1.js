/*
This is a mashup that uses the devices of the red cart.

If you cover the light sensor:
The robot arm will turn right, > will be displayed on a sensehat and the HUE light on the right will turn on.

If there is enough light on the light sensor, the opposite will happen.

If you press the joystick, the Dotstar LEDs will light up with random colors.
*/

const Robot_TD_ADDRESS = "http://192.168.0.105:8080/MeArmPi";
const LightSensor_TD_ADDRESS = "http://192.168.0.101/";
const senseHAT_TD_ADDRESS = "http://192.168.0.106:8080/SenseHat";
const strip_TD_ADDRESS = "http://192.168.0.103:8080/";
const HUE_1_ADDRESS = "file://../Devices/PhilipsHUE/lightTD1.json"
const HUE_3_ADDRESS = "file://../Devices/PhilipsHUE/lightTD3.json"

WoT.fetch(Robot_TD_ADDRESS).then(async (robotTD) => {

    WoT.fetch(senseHAT_TD_ADDRESS).then(async (senseHatTD) => {

        WoT.fetch(LightSensor_TD_ADDRESS).then(async (lightSensorTD) => {

            WoT.fetch(strip_TD_ADDRESS).then(async (stripTD) => {

                WoT.fetch(HUE_1_ADDRESS).then(async (hueLamp1TD) => {

                    WoT.fetch(HUE_3_ADDRESS).then(async (hueLamp3TD) => {

                        robotThing = WoT.consume(robotTD);
                        lightSensorThing = WoT.consume(lightSensorTD);
                        senseHatThing = WoT.consume(senseHatTD);
                        stripThing = WoT.consume(stripTD);
                        hueLamp1Thing = WoT.consume(hueLamp1TD);
                        hueLamp3Thing = WoT.consume(hueLamp3TD);

                        stripThing.actions["shutdown"].invoke();
                        senseHatThing.events.joystickPress.subscribe(x => {
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
                            if (intensity < 300) {
                                robotThing.actions["closeGrip"].invoke();
                                robotThing.actions["moveBaseTo"].invoke(35);
                                senseHatThing.actions["flashMessage"].invoke({
                                    "textString": "<"
                                });
                                hueLamp3Thing.actions["set_state"].invoke({
                                    "on": true,
                                    "bri": 250
                                });
                                hueLamp1Thing.actions["set_state"].invoke({
                                    "on": false
                                });
                            } else {
                                robotThing.actions["openGrip"].invoke();
                                robotThing.actions["moveBaseTo"].invoke(-35);
                                senseHatThing.actions["flashMessage"].invoke({
                                    "textString": ">"
                                });
                                hueLamp1Thing.actions["set_state"].invoke({
                                    "on": true,
                                    "bri": 250
                                });
                                hueLamp3Thing.actions["set_state"].invoke({
                                    "on": false
                                })
                            }
                        }, 200);
                    });
                });
            });
        });
    });
})