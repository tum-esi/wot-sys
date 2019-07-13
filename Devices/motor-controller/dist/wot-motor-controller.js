"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const os_1 = require("os");
var request = require('request');
var MotorController = require("../lib/MotorDriver.js").MotorDriver;
var controller = new MotorController();
class WotMotorController {
    constructor(thingFactory, tdDirectory) {
        this.factory = thingFactory;
        this.thing = thingFactory.produce(`{
            "@context": [
                "https://www.w3.org/2019/wot/td/v1",
                { "@language" : "en" }],
            "@type": "Thing",
            "title" : "MotorController",
            "description" : "Motor controller for pi",
            "securityDefinitions": { "nosec_sc": { "scheme": "nosec" }},
            "security": "nosec_sc"
        }`);
        while (!os_1.networkInterfaces().wlan0) { } // Wait for the network to get an IP address
        this.thing.id = "de:tum:ei:esi:motorcontroller:" + os_1.networkInterfaces().wlan0[0].address;
        this.add_properties();
        this.add_actions();
        this.thing.expose();
        if (tdDirectory) {
           // commented out for now this.register(tdDirectory);
        }
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

    getIsEnabled() {
        return new Promise((resolve, reject) => {
            resolve(controller.getEnabled());
        });
    }

    getIsFaulty() {
        return new Promise((resolve, reject) => {
            resolve(controller.getFaults());
        });
    }
    

    add_properties() {
        // Property: isEnabled
        this.thing.addProperty("isEnabled", {
            type: "string",
            readOnly: true,
            description: "Is both motors enabled?"
        });
        this.thing.setPropertyReadHandler("isEnabled", this.getIsEnabled);
        this.thing.addProperty("isFaulty", {
            type: "string",
            readOnly: true,
            description: "Is there any fault in motors?"
        });
        this.thing.setPropertyReadHandler("isFaulty", this.getIsFaulty);
        
    }
    add_actions() {
        this.thing.addAction("enable", { 
            description: "Enables the motor to move"  
        }, () => { 
            controller.enable(); 
            return "motors enabled";
        });

        this.thing.addAction("disable", { 
            description: "Disables the motors" 
        }, () => { 
            controller.disable(); 
            return "motors disabled";
        });
        this.thing.addAction("stop", { 
            description: "Stops the motors" 
        }, () => { 
            controller.stop(); 
            return "motors stopped";
        });
        this.thing.addAction("moveStraight", {
            description: "Moves the bot straigt (+ values to go forward - to go backward)",
            uriVariables: { "speed": {
                    "type": "integer",
                    "minimum": -255,
                    "maximum": 255
                } 
            }
        }, (data,options) => {
            if(options && 'uriVariables' in options) {
                let uriVariables = options['uriVariables'];
                if('speed' in uriVariables) {
                    let speed = uriVariables['speed'];
                    controller.goStraight(speed);
                    return "going straight with speed = " + speed;
                }
            }
            controller.goStraight(255);
            return "going straight full speed";
        });

        this.thing.addAction("turnLeft", {
            description: "Turning left with the given speed",
            uriVariables: { "speed": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 255
                } 
            }
        }, (data,options) => {
            if(options && 'uriVariables' in options) {
                let uriVariables = options['uriVariables'];
                if('speed' in uriVariables) {
                    let speed = uriVariables['speed'];
                    controller.turnLeft(speed);
                    return "Turning left with speed = " + speed;
                }
            }
            controller.turnLeft(255);
            return "Turning left full speed";
        });

        this.thing.addAction("turnRight", {
            description: "Turning right with the given speed",
            uriVariables: { "speed": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 255
                } 
            }
        }, (data,options) => {
            if(options && 'uriVariables' in options) {
                let uriVariables = options['uriVariables'];
                if('speed' in uriVariables) {
                    let speed = uriVariables['speed'];
                    controller.turnRight(speed);
                    return "Turning right with speed = " + speed;
                }
            }
            controller.turnRight(255);
            return "Turning right full speed";
        });

        this.thing.addAction("turnAround", {
            description: "Turning around with the given speed (+ for turn from right - for left)",
            uriVariables: { "speed": {
                    "type": "integer",
                    "minimum": -255,
                    "maximum": 255
                } 
            }
        }, (data,options) => {
            if(options && 'uriVariables' in options) {
                let uriVariables = options['uriVariables'];
                if('speed' in uriVariables) {
                    let speed = uriVariables['speed'];
                    controller.turnAround(speed);
                    return "Turning around with speed = " + speed;
                }
            }
            controller.turnAround(255);
            return "Turning around full speed";
        });

        this.thing.addAction("setSpeeds", {
            description: "giving speed values without changing the direction, which may also be used rather than move functions",
            uriVariables: { "leftSpeed": {
                    "type": "integer",
                    "minimum": -255,
                    "maximum": 255
                } ,"rightSpeed": {
                    "type": "integer",
                    "minimum": -255,
                    "maximum": 255
                } 
            }
        }, (data,options) => {
            if(options && 'uriVariables' in options) {
                let uriVariables = options['uriVariables'];
                if('leftSpeed' in uriVariables) {
                    let leftSpeed = uriVariables['leftSpeed'];
                    let rightSpeed = uriVariables['rightSpeed'];
                    controller.setSpeeds(leftSpeed,rightSpeed);
                    return "Motor speeds given left = " + leftSpeed + " right = " + rightSpeed;
                }
            }
            controller.setSpeeds(255,255);
            return "Full speed is given to motors";
        });
    }
}
exports.WotMotorController = WotMotorController;
//# sourceMappingURL=wot-mearmpi.js.map