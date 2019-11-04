//change if you have change js file
WotDevice = require("./dist/base.js").WotDevice

Servient = require("@node-wot/core").Servient

HttpServer = require("@node-wot/binding-http").HttpServer
//CoapServer = require("@node-wot/binding-coap").CoapServer
MqttBrokerServer = require("@node-wot/binding-mqtt").MqttBrokerServer

// Fill in the directory as link 
const TD_DIRECTORY = "http://192.168.0.100:8080/td"

//var coapServer = new CoapServer({port: 5683});
//var mqttserver = new MqttBrokerServer({uri:"test.mosquitto.org",port: 1883});
var httpServer = new HttpServer({port: 8080});
var servient = new Servient();

servient.addServer(httpServer);
//servient.addServer(coapServer);
//servient.addServer(mqttServer);
servient.start().then((thingFactory) => {
    pantilthat = new WotDevice(thingFactory, TD_DIRECTORY);
});
