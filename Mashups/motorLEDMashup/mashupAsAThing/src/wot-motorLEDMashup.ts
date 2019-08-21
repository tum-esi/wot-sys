import * as WoT from "wot-typescript-definitions"
//import { HttpServer } from "@node-wot/binding-http";
import { HttpClientFactory } from "@node-wot/binding-http";
//import { networkInterfaces } from "os"
var request = require('request');
var Servient = require("@node-wot/core").Servient

//var HttpServer = require("@node-wot/binding-http").HttpServer
//var MotorLEDMashup = require("../lib/motorLEDMashup").MotorLEDMashup;
//var mashup = new MotorLEDMashup();
const Motor_Driver_TD_ADDRESS = "http://192.168.0.113:8080/MotorController";
const strip_TD_ADDRESS = "http://192.168.0.103:8080/";

export class WotMotorLEDMashup {

    public ledThing : WoT.ConsumedThing;
    public motorThing : WoT.ConsumedThing;
    public blinkingStatus: boolean = false; //0 for not signaling 1 for signaling
    public blinkingDirection: boolean = false; //0 for left 1 for right

    public thing: WoT.ExposedThing;
    public factory: WoT.WoTFactory;

    constructor(thingFactory: WoT.WoTFactory, tdDirectory?: string) {
        this.factory = thingFactory;
        this.thing = thingFactory.produce(`{
            "@context": [
                "https://www.w3.org/2019/wot/td/v1",
                { "@language" : "en" }],
            "@type": "Thing",
            "title" : "MotorLEDMashup",
            "description" : "Motor LED strip mashup",
            "securityDefinitions": { "nosec_sc": { "scheme": "nosec" }},
            "security": "nosec_sc"
        }`);
        this.thing.id = "de:tum:ei:esi:motorledMashup:";// + networkInterfaces().wlan0[0].address;
        
        //var httpServer = new HttpServer({port: 8081});
        var servient = new Servient();
        //servient.addServer(httpServer);
        servient.addClientFactory(new HttpClientFactory());
        servient.start().then((thingFactory) => {
            console.log("!!!!!!!!!!!!!!  new sevient");
            thingFactory.fetch(Motor_Driver_TD_ADDRESS).then(async (motorTD) => {
                console.log("!!!!!!!!!!!!!!   fetch 1 done");
                this.motorThing =  thingFactory.consume(motorTD);
                thingFactory.fetch(strip_TD_ADDRESS).then(async (stripTD) => {
                    console.log("!!!!!!!!!!!!!!   fetch 2 done");
                    this.ledThing =  thingFactory.consume(stripTD);
                });
            });
        });
        
        this.add_properties();
        this.add_actions();
        this.thing.expose();
        console.log("!!!!!!!!!!!!!!   expose done");
        if (tdDirectory) { this.register(tdDirectory); }
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
        })
    }
    private get_nonld_td() {
        let td = JSON.parse(this.thing.getThingDescription());
        delete td["@context"];
        delete td["@type"];
        return td;
    }

    private add_properties() {
        // Property: gripState
        this.thing.addProperty(
            "isBlinking",
            {
                type: "string", 
                readOnly: true,
                description: "Is any of the blinker leds enabled?", 
            }
        );
        this.thing.setPropertyReadHandler("isBlinking", this.getIsBlinking)
    }    
    private add_actions() {
        this.thing.addAction(
            "stop", 
            {description: "Stops the motors"}, 
            () => { return this.stop(); }
        );
        this.thing.addAction(
            "turnLeft", 
            {description: "Turning left if signals are on else opens signal"}, 
            () => { 
                if(this.blinkingStatus) {
                    this.stopBlinking();
                    return this.turnLeft();
                } else {
                    return this.blinkLeft();
                }
            }
        );
        this.thing.addAction(
            "turnRight", 
            {description: "Turning right if signals are on else opens signal"}, 
            () => { 
                if(this.blinkingStatus) {
                    this.stopBlinking();
                    return this.turnRight();
                } else {
                    return this.blinkRight();
                }
            }
        );
        this.thing.addAction(
            "goStraight",{
                description: "Moves the bot straigt (+ values to go forward - to go backward)",
                uriVariables: { "speed": 
                    {
                        "type": "integer",
                        "minimum": -255,
                        "maximum": 255
                    } 
                }
            },
            (speed) => { return this.goStraight(speed); }
        );
    }

    private getIsBlinking () {
        return new Promise((resolve, reject) => {
            let status = this.blinkingStatus;
            if (status) {
                resolve("blinking");
            } else {
                resolve("not blinking");
            }    
        });
    }
    

    private goStraight(speed) {
        return new Promise((resolve, reject) => {
            if (speed > 255 || speed < -255){
                resolve("wrong input");
            } else {
                this.motorThing.actions["moveStraight"].invoke({
                    "speed": speed
                 });
                 resolve("going straight");
            }
        });
    }

    private stop() {
        return new Promise((resolve, reject) => {
            this.motorThing.actions["stop"].invoke();
        });
    }
    private turnLeft() {
        return new Promise((resolve, reject) => {
            this.motorThing.actions["turnLeft"].invoke();
            resolve("Turning left full speed");
        });
    }
    private turnRight() {
        return new Promise((resolve, reject) => {
            this.motorThing.actions["turnRight"].invoke();
            resolve("Turning right full speed");
        });
    }

    // blink functions
    private blinkRight () {
        return new Promise((resolve, reject) => {
            this.blinkingStatus = true;
            this.openLedsYellow(12,18)
            this.blinkingDirection = true;
            resolve("Blinking right");
        });
    }

    private blinkLeft () {
        return new Promise((resolve, reject) => {
            this.blinkingStatus = true;
            this.openLedsYellow(23,29)
            this.blinkingDirection = false;
            resolve("Blinking left");
        });
    }

    private stopBlinking () {
        return new Promise((resolve, reject) => {
            this.closeLeds();
            this.blinkingStatus = false;
            resolve("blink closed");
        });
    }

    private closeLeds() {
        var ledBegin = 23;
        var ledEnd = 29;
        if(this.blinkingDirection == true) {//right
            ledBegin = 12;
            ledEnd = 18;
        }
        this.ledThing.actions["fill_array"].invoke({
            "ledBegin":ledBegin,
            "ledEnd":ledEnd,
            "color":{
                "red": 0,
                "green" : 0,
                "blue" : 0
            }
        });
    }

    private openLedsYellow( ledBegin, ledEnd) {
        this.ledThing.actions["fill_array"].invoke({
            "ledBegin":ledBegin,
            "ledEnd":ledEnd,
            "color":{
                "red": 255,
                "green" : 106,
                "blue" : 0
            }
        });
    }
}