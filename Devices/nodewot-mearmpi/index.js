MeArmPi = require("./dist/wot-mearmpi").WotMeArmPi
Servient = require("@node-wot/core").Servient
HttpServer = require("@node-wot/binding-http").HttpServer


// ### Change this if using another direcoty ###
const TD_DIRECTORY = "http://192.168.0.100:8080/td"
// ###  -----------------------------------  ###


var httpServer = new HttpServer({port: 8080});
var servient = new Servient();

servient.addServer(httpServer);
servient.start().then((thingFactory) => {
    arm = new MeArmPi(thingFactory, TD_DIRECTORY);
})
