import * as WoT from "wot-typescript-definitions"
import { HttpClientFactory } from "@node-wot/binding-http";
var request = require('request');
var Servient = require("@node-wot/core").Servient
export class WotDevice {
    public thing: WoT.ExposedThing;
    public factory: WoT.WoTFactory;
    constructor(thingFactory: WoT.WoTFactory, tdDirectory?: string) {
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
            }`
            );
        this.thing.id = "new:thing";
        this.add_properties();
        this.add_actions();
        this.thing.expose();
        console.log("thing expose done");
        if (tdDirectory) { this.register(tdDirectory); }
        var servient = new Servient();
        servient.addClientFactory(new HttpClientFactory());
        servient.start().then((thingFactory) => {
            //fetch sub things

        });
    }
    public register(directory: string) {
        console.log("Registering TD in directory: " + directory)
        request.post(directory, {json: this.get_nonld_td()}, (error, response, body) => {
            if (!error && response.statusCode < 300) {
                console.log("TD registered!");
            } else {
                console.debug(error);
                console.debug(response);
                console.warn("Failed to register TD. Will try again in 10 Seconds...");
                setTimeout(() => { this.register(directory) }, 10000);
                return;
            }
        });
    }
    private get_nonld_td() {
        let td = JSON.parse(this.thing.getThingDescription());
        delete td["@context"];
        delete td["@type"];
        return td;
    }
    private add_properties() {
        //fill in add properties
        //this.thing.addProperty
    }
    private add_actions() {
        //fill in add actions
        //  this.thing.addAction(
    }
}