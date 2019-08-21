/*
This is a mashup that uses the devices of the red cart.

If you cover the light sensor:
The robot arm will turn right, > will be displayed on a sensehat and the HUE light on the right will turn on.

If there is enough light on the light sensor, the opposite will happen.

If you press the joystick, the Dotstar LEDs will light up with random colors.
*/

const Robot_TD_ADDRESS = "http://192.168.0.104:8080/MeArmPi";
WoT.fetch(Robot_TD_ADDRESS).then(async (robotTD) => {

    robotThing = WoT.consume(robotTD);

    setInterval(async () => {

        robotThing.actions["closeGrip"].invoke();
        robotThing.actions["dance"].invoke().then(()=>{
robotThing.actions["openGrip"].invoke();
        })
        
    }, 20000);
});