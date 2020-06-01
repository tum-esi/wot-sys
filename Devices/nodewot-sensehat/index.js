WotSenseHat = require("./dist/sensehat").WotSenseHat
Servient = require("@node-wot/core").Servient
HttpServer = require("@node-wot/binding-http").HttpServer


// ### Change this if using another direcoty ###
const TD_DIRECTORY = "http://172.16.1.100:8080/api/td"
// ###  -----------------------------------  ###


var httpServer = new HttpServer({port: 8080});
var servient = new Servient();

servient.addServer(httpServer);
servient.start().then((WoT) => {
    hat = new WotSenseHat(WoT, TD_DIRECTORY);
})
