/*
A Mashup that indicates if the room is hot or not by moving the robot to a side and printing a message. If the robot moved clockwise then it's cold otherwise it's hot.
This can also be read on the pixels-Screen of SenseHat. The greater then sign means it's hot and the less sign means it's cold.
*/
const Robot_TD_ADDRESS = "http://TUMEIESI-MeArmPi-Orange.local:8080/MeArmPi";
const senseHAT_TD_ADDRESS = "http://TUMEIESI-SenseHat-106.local:8080/SenseHat/";

WoT.fetch(Robot_TD_ADDRESS).then(async (robotTD) => {

    WoT.fetch(senseHAT_TD_ADDRESS).then(async (senseHatTD) => {

                        robotThing = WoT.consume(robotTD);
                        senseHatThing = WoT.consume(senseHatTD);
                        
                        robotThing.actions["dance"].invoke();

                        setInterval(async () => {
                            var temperature =  await senseHatThing.properties.temperature.read();
                            console.log(temperature);
                            if (temperature < 33) {
                                robotThing.actions["moveBaseTo"].invoke(30);
                                senseHatThing.actions["flashMessage"].invoke({
                                    "textString": "<",
                                    "scrollSpeed": 400
                                });
                            } else {
                                robotThing.actions["openGrip"].invoke();
                                robotThing.actions["moveBaseTo"].invoke(-30);
                                senseHatThing.actions["flashMessage"].invoke({
                                    "textString": ">",
                                    "scrollSpeed": 400
                                });
                            }
                        }, 5000);
        
            });
        });
