/*
This Mashup lights the DoStar LEDs blue  if the temperature is low and red if the Temperature is highs
*/
const Robot_TD_ADDRESS = "http://TUMEIESI-MeArmPi-Orange.local:8080/MeArmPi";
const LightSensor_TD_ADDRESS = "http://192.168.188.8";
const senseHAT_TD_ADDRESS = "http://TUMEIESI-SenseHat-107.local:8080/SenseHat/";
const DotStar_TD_ADDRESS = "http://TUMEIESI-DotStar.local:8080/";
const strip_TD_ADDRESS = "http://192.168.0.103:8080/";
const HUE_1_ADDRESS = "file://../Devices/PhilipsHUE/lightTD1.json";
const HUE_3_ADDRESS = "file://../Devices/PhilipsHUE/lightTD3.json";
const Light1_TD_ADDRESS = "http://192.168.0.111/api/R6D7CYQFzXckikMPLEL8WbSZWg9XKkEyx-NrgKws/lights/1/";


WoT.fetch(SenseHat_TD_ADDRESS).then((SenseHatTD) => {

    WoT.fetch(DotStar_TD_ADDRESS).then((DotStarTD) => {

        SenseHat = WoT.consume(SenseHatTD);
        DotStar = WoT.consume(DotStarTD);
        setInterval(() => {
            SenseHat.properties.temperature.read().then((tmp)=>{
                red = Math.min(255,(tmp -40) * 255/5);
                blue = Math.min(255,(45-tmp) * 255/5);
                DotStar.actions["fill"].invoke({
                    "green": 0,
                    "red": red,
                    "blue": blue
                });
            }).catch((err) => console.log(err))
        }, 2000);
    }).catch((err) => console.log(err))
}).catch((err) => console.log(err))


