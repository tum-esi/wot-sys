WotDevice = require("./dist/base.js").WotDevice
Servient = require("@node-wot/core").Servient
HttpServer = require("@node-wot/binding-http").HttpServer
MqttBrokerServer = require("@node-wot/binding-mqtt").MqttBrokerServer

// Fill in the directory as link 
const TD_DIRECTORY = "http://192.168.0.100:8080/td"

var mqttServer = new MqttBrokerServer("test.mosquitto.org");
//var mqttServer = new MqttBrokerServer("10.181.206.241");
var httpServer = new HttpServer({port: 8080});
var servient = new Servient();

servient.addServer(httpServer);
servient.addServer(mqttServer);
servient.start().then((WoT) => {
    pantilthat = new WotDevice(WoT, TD_DIRECTORY);
});
