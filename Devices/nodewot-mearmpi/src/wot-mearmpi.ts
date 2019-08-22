import * as WoT from "wot-typescript-definitions"
import { networkInterfaces } from "os"

var request = require('request');
var MeArmPi = require("../lib/MeArmPi.js").MeArmPi;
var arm = new MeArmPi();


export class WotMeArmPi {
    public thing: WoT.ExposedThing;
    public factory: WoT.WoTFactory;

    constructor(thingFactory: WoT.WoTFactory, tdDirectory?: string) {
        this.factory = thingFactory;
        this.thing = thingFactory.produce(`{
            "@context": [
                "https://www.w3.org/2019/wot/td/v1",
                { "@language" : "en" }],
            "@type": "Thing",
            "title" : "MeArmPi",
            "description" : "MeArm Pi Robotic Arm",
            "securityDefinitions": { "nosec_sc": { "scheme": "nosec" }},
            "security": "nosec_sc"
        }`);
        while (!networkInterfaces().wlan0) { }  // Wait for the network to get an IP address
        this.thing.id = "de:tum:ei:esi:mearmpi:" + networkInterfaces().wlan0[0].address;
        this.add_properties();
        this.add_actions();
        this.thing.expose();
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

    private get_grip_state() {
        return new Promise((resolve, reject) => {
            let angle = arm.servoState().grip;
            let state = "between";
            if (angle === 90) state = "open"
            if (angle === 0) state = "closed"
            resolve(state);
         });
    }

    private get_motor_positions() {
        return new Promise((resolve, reject) => {
            resolve(arm.servoState());
         });
    }

    private move_to(angles) {
        return new Promise((resolve, reject) => {
            new Promise((resolve, reject) => {
                if (typeof(angles.base) === "number") {
                    arm.moveBaseTo(angles.base, (msg)=>{if (msg === "complete") resolve();}); 
                } else { resolve() }
            })
            .then(() => { return new Promise((resolve, reject) => {
                if (typeof(angles.lower) === "number") {
                    arm.moveLowerTo(angles.lower, (msg)=>{if (msg === "complete") resolve();});
                } else { resolve() }
            })})
            .then(() => { return new Promise((resolve, reject) => {
                if (typeof(angles.upper) === "number") {
                    arm.moveUpperTo(angles.upper, (msg)=>{if (msg === "complete") resolve();});
                } else { resolve() }
            })})
            .then(() => { return new Promise((resolve, reject) => {
                if (typeof(angles.grip) === "number") {
                    arm.moveGripTo(angles.grip, (msg)=>{if (msg === "complete") resolve();});
                } else { resolve() }
            })})
            .then(() => resolve())
            .catch(() => reject())
         }); 
    }

    private dance() {
        return new Promise((resolve, reject) => {
            this.move_to({base:0, lower:90, upper:90, grip:45})
            .then(() => { return this.move_to({base:85, lower:110, upper:130, grip:0}); })
            .then(() => { return this.move_to({upper:20}); })
            .then(() => { return this.move_to({grip:90}); })
            .then(() => { return this.move_to({grip:0}); })
            .then(() => { return this.move_to({base:-85, lower:50, upper: 110}); })
            .then(() => { return this.move_to({grip:90}); })
            .then(() => { return this.move_to({grip:0}); })
            .then(() => { return this.move_to({base:0, lower:90, upper:90, grip:45}); })
            .then(() => resolve())
            .catch(() => reject()) 
         });    
    }

    private add_properties() {
        // Property: gripState
        this.thing.addProperty(
            "gripState",
            {
                type: "string", 
                readOnly: true,
                description: "Is the robots grip open or closed?", 
                enum: ["open", "closed", "between"]
            }
        );
        this.thing.setPropertyReadHandler("gripState", this.get_grip_state)
        
        // Property: motorPositions
        this.thing.addProperty(
            "motorPositions",
            {
                type: "object", 
                readOnly: true,
                description: "The current positions of all 4 motors.", 
                properties: {
                    "base": {
                        "type": "integer"
                    }, 
                    "upper": {
                        "type": "integer"
                    }, 
                    "lower": {
                        "type": "integer"
                    },
                    "grip": {
                        "type": "integer"
                    }
                }
            }
        );
        this.thing.setPropertyReadHandler("motorPositions", this.get_motor_positions)
    }

    private add_actions() {
        this.thing.addAction(
            "openGrip", 
            {description: "Open the robot's Grip."}, 
            () => { return this.move_to({grip: 90}); }
        );
        this.thing.addAction(
            "closeGrip", 
            {description: "Close the robot's Grip."}, 
            () => { return this.move_to({grip: 0}); }
        );
        this.thing.addAction(
            "moveGripTo",
            {
                description: "Move the grip's to a given angle.",
                input: {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 90
                }
            },
            (angle) => { return this.move_to({grip: angle}); }
        );
        this.thing.addAction(
            "moveUpperTo",
            {
                description: "Move the upper part of the arm to a given angle.",
                input: {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 135
                }
            },
            (angle) => { return this.move_to({upper: angle}); }
        );
        this.thing.addAction(
            "moveLowerTo",
            {
                description: "Move the lower part of the arm to a given angle.",
                input: {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 135
                }
            },
            (angle) => { return this.move_to({lower: angle}); }
        );
        this.thing.addAction(
            "moveBaseTo",
            {
                description: "Move the base of the arm to a given angle.",
                input: {
                    "type": "integer",
                    "minimum": -90,
                    "maximum": 90
                }
            },
            (angle) => { return this.move_to({base: angle}); }
        );
        this.thing.addAction(
            "dance",
            { },
            () => { return this.dance() }
        );
    }
}