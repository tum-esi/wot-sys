EnviroPHatPi = require("./dist/wot-envirophatpi").WoTEnviroPHatPi 
Servient = require("@node-wot/core").Servient
HttpServer = require("@node-wot/binding-http").HttpServer
CoapServer = require("@node-wot/binding-coap").CoapServer


// ### change this if using another directory ###
const TD_DIRECTORY = "http://192.168.0.100:8080/td"
// ### -------------------------------------- ###


var httpServer = new HttpServer({port: 8080});
var coapServer = new CoapServer({port: 5683});
var servient = new Servient();

servient.addServer(httpServer);
servient.addServer(coapServer);
servient.start().then((thingFactory) => {
	enviro_phat = new EnviroPHatPi(thingFactory,TD_DIRECTORY);
})
