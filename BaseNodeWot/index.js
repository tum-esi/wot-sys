//change if you have change js file
WotDevice = require("./dist/base.js").WotDevice
Servient = require("@node-wot/core").Servient
HttpServer = require("@node-wot/binding-http").HttpServer
//CoapServer = require("@node-wot/binding-coap").CoapServer
//MQTTServer = require("@node-wot/binding-mqtt").MqttServer

// Fill in the directory as link 
const TD_DIRECTORY = ""

//var coapServer = new CoapServer({port: 5683});
//var mqttserver = new MQTTServer({port: 1234}):
var httpServer = new HttpServer({port: 8080});
var servient = new Servient();

servient.addServer(httpServer);
//servient.addServer(coapServer);
//servient.addServer(mqttServer);
servient.start().then((thingFactory) => {
    motorController = new WotDevice(thingFactory, TD_DIRECTORY);
});
