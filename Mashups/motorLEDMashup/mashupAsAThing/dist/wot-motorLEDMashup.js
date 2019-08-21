"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : new P(function (resolve) { resolve(result.value); }).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
Object.defineProperty(exports, "__esModule", { value: true });
const binding_http_1 = require("@node-wot/binding-http");
var request = require('request');
var Servient = require("@node-wot/core").Servient;
const Motor_Driver_TD_ADDRESS = "http://192.168.0.113:8080/MotorController";
const strip_TD_ADDRESS = "http://192.168.0.103:8080/";
class WotMotorLEDMashup {
    constructor(thingFactory, tdDirectory) {
        this.blinkingStatus = false; //0 for not signaling 1 for signaling
        this.blinkingDirection = false; //0 for left 1 for right
        //create mashup as a server
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
        this.thing.id = "de:tum:ei:esi:motorledMashup:";
        this.add_properties();
        this.add_actions();
        this.thing.expose();
        console.log("!!!!!!!!!!!!!!   expose done");
        if (tdDirectory) {
            this.register(tdDirectory);
        }
        //consume LED and Motor controller
        var servient = new Servient();
        servient.addClientFactory(new binding_http_1.HttpClientFactory());
        servient.start().then((thingFactory) => {
            console.log("!!!!!!!!!!!!!!  new sevient");
            thingFactory.fetch(Motor_Driver_TD_ADDRESS).then((motorTD) => __awaiter(this, void 0, void 0, function* () {
                console.log("!!!!!!!!!!!!!!   fetch 1 done");
                this.motorThing = thingFactory.consume(motorTD);
                thingFactory.fetch(strip_TD_ADDRESS).then((stripTD) => __awaiter(this, void 0, void 0, function* () {
                    console.log("!!!!!!!!!!!!!!   fetch 2 done");
                    this.ledThing = thingFactory.consume(stripTD);
                }));
            }));
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
        this.thing.addProperty("isBlinking", {
            type: "string",
            readOnly: true,
            description: "Is any of the blinker leds enabled?",
        });
        this.thing.setPropertyReadHandler("isBlinking", this.getIsBlinking);
    }
    add_actions() {
        this.thing.addAction("stop", { description: "Stops the motors" }, () => { return this.stop(); });
        this.thing.addAction("turnLeft", { description: "Turning left if signals are on else opens signal" }, () => {
            if (this.blinkingStatus) {
                this.stopBlinking();
                return this.turnLeft();
            }
            else {
                return this.blinkLeft();
            }
        });
        this.thing.addAction("turnRight", { description: "Turning right if signals are on else opens signal" }, () => {
            if (this.blinkingStatus) {
                this.stopBlinking();
                return this.turnRight();
            }
            else {
                return this.blinkRight();
            }
        });
        this.thing.addAction("goStraight", {
            description: "Moves the bot straigt (+ values to go forward - to go backward)",
            uriVariables: { "speed": {
                    "type": "integer",
                    "minimum": -255,
                    "maximum": 255
                }
            }
        }, (speed) => { return this.goStraight(speed); });
    }
    getIsBlinking() {
        return new Promise((resolve, reject) => {
            let status = this.blinkingStatus;
            if (status) {
                resolve("blinking");
            }
            else {
                resolve("not blinking");
            }
        });
    }
    //motor thing related functions
    goStraight(speed) {
        return new Promise((resolve, reject) => {
            if (speed > 255 || speed < -255) {
                resolve("wrong input");
            }
            else {
                this.motorThing.actions["moveStraight"].invoke({
                    "speed": speed
                });
                resolve("going straight");
            }
        });
    }
    stop() {
        return new Promise((resolve, reject) => {
            this.motorThing.actions["stop"].invoke();
            resolve("Stopped");
        });
    }
    turnLeft() {
        return new Promise((resolve, reject) => {
            this.motorThing.actions["turnLeft"].invoke();
            resolve("Turning left full speed");
        });
    }
    turnRight() {
        return new Promise((resolve, reject) => {
            this.motorThing.actions["turnRight"].invoke();
            resolve("Turning right full speed");
        });
    }
    // blink functions
    blinkRight() {
        return new Promise((resolve, reject) => {
            this.blinkingStatus = true;
            this.openLedsYellow(12, 18);
            this.blinkingDirection = true;
            resolve("Blinking right");
        });
    }
    blinkLeft() {
        return new Promise((resolve, reject) => {
            this.blinkingStatus = true;
            this.openLedsYellow(23, 29);
            this.blinkingDirection = false;
            resolve("Blinking left");
        });
    }
    stopBlinking() {
        return new Promise((resolve, reject) => {
            this.closeLeds();
            this.blinkingStatus = false;
            resolve("blink closed");
        });
    }
    //Led thing related functions
    closeLeds() {
        var ledBegin = 23;
        var ledEnd = 29;
        if (this.blinkingDirection == true) { //right
            ledBegin = 12;
            ledEnd = 18;
        }
        this.ledThing.actions["fill_array"].invoke({
            "ledBegin": ledBegin,
            "ledEnd": ledEnd,
            "color": {
                "red": 0,
                "green": 0,
                "blue": 0
            }
        });
    }
    openLedsYellow(ledBegin, ledEnd) {
        this.ledThing.actions["fill_array"].invoke({
            "ledBegin": ledBegin,
            "ledEnd": ledEnd,
            "color": {
                "red": 255,
                "green": 106,
                "blue": 0
            }
        });
    }
}
exports.WotMotorLEDMashup = WotMotorLEDMashup;
//# sourceMappingURL=wot-motorLEDMashup.js.map