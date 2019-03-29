const TD_ADDRESS = "http://192.168.0.12:8080/SenseHat"

WoT.fetch(TD_ADDRESS).then((td) => {

    thing = WoT.consume(td);

    thing.actions["showMessage"].invoke({"textString": "Hello World!"})

});