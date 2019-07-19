/*
This Mashup lights an  light, that means the robot is ready to take an object and close its grip. Seconds after that the robot closes the grip and moves
in a circular way to the oposite side (I.e. moves 140 degrees to the right). The robot open the grip to give the person on the other side the object and 
returs back to its first state. This movement/sequence of action is iterative.
*/
const Robot_TD_ADDRESS = "http://TUMEIESI-MeArmPi-Orange.local:8080/MeArmPi";
const DotStar_TD_ADDRESS = "http://TUMEIESI-DotStar.local:8080/";

WoT.fetch(Robot_TD_ADDRESS).then(async (robotTD) => {

    WoT.fetch(DotStar_TD_ADDRESS).then(async (DotStarTD) => {

        
        DotStar = WoT.consume(DotStarTD);
        robotThing = WoT.consume(robotTD);
        setInterval(async () => {
            DotStar.actions["fill"].invoke({
                "green": 255,
                "red": 0,
                "blue": 0
            });
            setTimeout(function () {
                DotStar.actions["shutdown"].invoke();
                setTimeout(function () {
                    robotThing.actions["closeGrip"].invoke();
                    setTimeout(function () {
                        robotThing.actions["moveBaseTo"].invoke(70);
                        setTimeout(function () {
                            robotThing.actions["openGrip"].invoke();
                            setTimeout(function () {
                                robotThing.actions["moveBaseTo"].invoke(-70);
                            }, 3000)
                        }, 3000)
                    }, 3000)
                }, 3000)
            }, 3000)
        }, 25000);
    });
});
