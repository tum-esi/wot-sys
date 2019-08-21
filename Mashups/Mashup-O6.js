/*
This Mashup involves divices with "id":"de:tum:ei:esi:mearmpi:192.168.0.104", "id": "de:tum:ei:esi:dotstar:192.168.0.103:8080" 
and "id":"de:tum:ei:esi:sensehat:192.168.0.106".

This Mashup does a random act for 5 seconds when the button on Sensehat is pressed.
*/
const Robot_TD_ADDRESS = "http://TUMEIESI-MeArmPi-Blue.local:8080/MeArmPi";
const senseHAT_TD_ADDRESS = "http://TUMEIESI-SenseHat-106.local:8080/SenseHat/";
const DotStar_TD_ADDRESS = "http://TUMEIESI-DotStar.local:8080/";

WoT.fetch(Robot_TD_ADDRESS).then(async (robotTD) => {

    WoT.fetch(DotStar_TD_ADDRESS).then(async (DotStarTD) => {


        WoT.fetch(senseHAT_TD_ADDRESS).then(async (senseHatTD) => {

            DotStar = WoT.consume(DotStarTD);
            robotThing = WoT.consume(robotTD);
            senseHatThing = WoT.consume(senseHatTD);

            var index1 = 0;
            var indexBefore = 0;
            var indexAfter = 0;

            setInterval( () => {
                if (indexAfter > indexBefore){
                    index1=Math.floor(Math.random() * 7);
                    console.log(index1);
                }
                console.log("begeniin" + index1);
                console.log("Happened")
                if (index1 == -1) {
                    console.log("at -1");
                    DotStar.actions["shutdown"].invoke();
                }
                    else if (index1 == 0) {
                        console.log("at 0");
                        DotStar.actions["dot"].invoke({
                            "led": 24,
                            "color": {
                                "red": 255,
                                "green": 255,
                                "blue": 255
                            }
                        });
                    }
                    else if (index1 == 1) {
                        console.log("at 1");
                        DotStar.actions["fill"].invoke({
                            "red": 255,
                            "green": 0,
                            "blue": 0
                        });
                    }
                    else if (index1 == 2) {
                        console.log("at 2");
                        DotStar.actions["fill"].invoke({
                            "red": 0,
                            "green": 255,
                            "blue": 0
                        });
                    }
                    else if (index1 == 3) {
                        console.log("at 3");
                        DotStar.actions["fill"].invoke({
                            "red": 0,
                            "green": 0,
                            "blue": 255
                        });
                    }
                    else if (index1 == 4) {
                        console.log("at 4");
                        DotStar.actions["random"].invoke();
                    }
                    else {
                        robotThing.actions["dance"].invoke();
                        DotStar.actions["random"].invoke();
                    }
                    senseHatThing.events.joystickPress.subscribe(z => {
                        indexAfter++
                    },
                        e => {
                            console.log("onError: %s", e)
                        },
                        () => {
                            console.log("onCompleted");
                        }
                    )
                    indexBefore = indexAfter
                    index1 = -1;
            }, 5000).catch((err) => console.log(err))
        });
    });
});
