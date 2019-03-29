const TD_ADDRESS = "http://192.168.0.105:8080/MeArmPi"

WoT.fetch(TD_ADDRESS).then((td) => {

    thing = WoT.consume(td);

    thing.actions["moveBaseTo"].invoke(75)
    thing.actions["closeGrip"].invoke()
});

