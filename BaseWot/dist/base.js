"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const binding_http_1 = require("@node-wot/binding-http");
var request = require('request');
var Servient = require("@node-wot/core").Servient;
class WotDevice {
    constructor(thingFactory, tdDirectory) {
        //create WotDevice as a server
        this.factory = thingFactory;
        this.thing = thingFactory.produce(
        //fill in the empty quotation marks
        `{
                "@context": [
                    "https://www.w3.org/2019/wot/td/v1",
                    { "@language" : "en" }],
                "@type": "",
                "title" : "",
                "description" : "",
                "securityDefinitions": { 
                    "": { 
                        "scheme": "" 
                    }
                },
                "security": ""
            }`);
        this.thing.id = "new:thing";
        this.add_properties();
        this.add_actions();
        this.thing.expose();
        console.log("thing expose done");
        if (tdDirectory) {
            this.register(tdDirectory);
        }
        var servient = new Servient();
        servient.addClientFactory(new binding_http_1.HttpClientFactory());
        servient.start().then((thingFactory) => {
            //fetch sub things
        });
    }
    register(directory) {
        console.log("Registering TD in directory: " + directory);
        request.post(directory, { json: this.get_nonld_td() }, (error, response, body) => {
            if (!error && response.statusCode < 300) {
                console.log("TD registered!");
            }
            else {
                console.debug(error);
                console.debug(response);
                console.warn("Failed to register TD. Will try again in 10 Seconds...");
                setTimeout(() => { this.register(directory); }, 10000);
                return;
            }
        });
    }
    get_nonld_td() {
        let td = JSON.parse(this.thing.getThingDescription());
        delete td["@context"];
        delete td["@type"];
        return td;
    }
    add_properties() {
        //fill in add properties
        //this.thing.addProperty
    }
    add_actions() {
        //fill in add actions
        //  this.thing.addAction(
    }
}
exports.WotDevice = WotDevice;
//# sourceMappingURL=base.js.map