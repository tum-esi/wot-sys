/*
A Mashup that lights all pixels of SenseHAT IN RED COLOR. Then everytime the button is pressed one pixel lights in white.
*/
const senseHAT_TD_ADDRESS = "http://TUMEIESI-SenseHat-106.local:8080/SenseHat/";


    WoT.fetch(senseHAT_TD_ADDRESS).then(async (senseHatTD) => {




                        senseHatThing = WoT.consume(senseHatTD);

                        let arrayx = [255,0,0]
                        var array = []
                        i=0;
                        while(i<64){
                            array.push(arrayx)
                            i++;
                        }
                        var indexBefore = 0;
                        var indexAfter = 0;
                        var index1 = 0;
                        setInterval(async () => {
                            if (index1 == 0){
                                for (i = 0;i < 64;i++){
                                    let xPar = Math.floor(i/8);
                                    let yPar= i % 8;
                                    senseHatThing.actions["setPixel"].invoke({
                                        "x":xPar,
                                        "y":yPar,
                                        "r": 255,
                                        "g": 0,
                                        "b": 0
                                    });
                                }
                            }
                        senseHatThing.events.joystickPress.subscribe(z => {
                            console.log(index1)
                            var xValue =Math.floor(index1/8);
                            var yValue= index1 % 8;
                            senseHatThing.actions["setPixel"].invoke({
                                "x":xValue,
                                "y":yValue,
                                "r": 128,
                                "g": 128,
                                "b": 128
                            });
                            indexAfter++
                            if (index1 == 63) {
                                index1 = 0
                                senseHatThing.actions["clear"].invoke()
                            }
                        },
                            e => {
                                console.log("onError: %s", e)
                            },
                            () => {
                                console.log("onCompleted");
                            }
                        )
                        if (indexAfter > indexBefore){
                            index1++
                        }
                        indexBefore = indexAfter
                    },300).catch((err) => console.log(err))
                    });

