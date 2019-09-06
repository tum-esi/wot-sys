//change if you have change js file
WotDevice = require("./dist/base.js").WotDevice
Servient = require("@node-wot/core").Servient
HttpServer = require("@node-wot/binding-http").HttpServer


// Fill in the directory as link 
const TD_DIRECTORY = ""



var httpServer = new HttpServer({port: 8080});
var servient = new Servient();

servient.addServer(httpServer);
servient.start().then((thingFactory) => {
    motorController = new WotDevice(thingFactory, TD_DIRECTORY);
});
