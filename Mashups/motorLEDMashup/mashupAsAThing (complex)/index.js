MotorLEDMashup = require("./dist/wot-motorLEDMashup").WotMotorLEDMashup
Servient = require("@node-wot/core").Servient
HttpServer = require("@node-wot/binding-http").HttpServer


// ### Change this if using another direcoty ###
const TD_DIRECTORY = "http://192.168.0.100:8080/td"
// ###  -----------------------------------  ###


var httpServer = new HttpServer({port: 8082});
var servient = new Servient();

servient.addServer(httpServer);
servient.start().then((thingFactory) => {
    mashup = new MotorLEDMashup(thingFactory, TD_DIRECTORY);
})
