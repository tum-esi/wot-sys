"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const request = require('request');
const sense = require("sense-hat-led");
const nodeimu = require("nodeimu");
const imu = new nodeimu.IMU();
const sense_joystick = require("sense-joystick");
const Ajv = require('ajv');
var ajv = new Ajv();
class WotSenseHat {
    constructor(WoT, tdDirectory) {
        this.WoT = WoT;
        this.WoT.produce({
            "@context": [
                "https://www.w3.org/2019/wot/td/v1",
                { "@language": "en" }
            ],
            "title": "SenseHat",
            "description": "Raspberry Pi SenseHat",
            "securityDefinitions": { "nosec_sc": { "scheme": "nosec" } },
            "security": "nosec_sc",
            "properties": {
                "displayRotation": {
                    type: "integer",
                    readOnly: false,
                    writeOnly: false,
                    description: "The rotation of the display. 0 is with the HDMI port facing down.",
                    enum: [0, 90, 180, 270]
                },
                "pixels": {
                    type: "array",
                    minItems: 64,
                    maxItems: 64,
                    items: {
                        type: "array",
                        items: { "type": "integer", "minimum": 0, "maximum": 255 },
                        minItems: 3,
                        maxItems: 3,
                    },
                    readOnly: false,
                    writeOnly: false,
                    description: "An array containing the current colours of all LEDs.",
                },
                "lowLight": {
                    type: "boolean",
                    readOnly: false,
                    writeOnly: false,
                    description: "The LED Matrix low light mode.",
                },
                "humidity": {
                    type: "number",
                    description: "The percentage of relative humidity.",
                    readOnly: true,
                    observable: true
                },
                "temperature": {
                    type: "number",
                    description: "The current temperature in degrees Celsius.",
                    readOnly: true,
                    observable: true
                },
                "pressure": {
                    type: "number",
                    description: "The current pressure in Millibars.",
                    readOnly: true,
                    observable: true
                },
                "gyro": {
                    type: "object",
                    description: "Get 3 Floats representing the rotational intensity of each axis in rads/second.",
                    readOnly: true,
                    observable: true,
                    properties: {
                        "x": { "type": "number" },
                        "y": { "type": "number" },
                        "z": { "type": "number" }
                    }
                },
                "acceleration": {
                    type: "object",
                    description: "Get 3 Floats representing the acceleration intensity of each axis in Gs",
                    readOnly: true,
                    observable: true,
                    properties: {
                        "x": { "type": "number" },
                        "y": { "type": "number" },
                        "z": { "type": "number" }
                    }
                },
                "compass": {
                    type: "object",
                    description: "Get 3 Floats representing the magnetic intensity of the axis in microteslas.",
                    readOnly: true,
                    observable: true,
                    properties: {
                        "x": { "type": "number" },
                        "y": { "type": "number" },
                        "z": { "type": "number" }
                    }
                },
                "eventHistory": {
                    type: "array",
                    description: "An array containing the complete event history.",
                    readOnly: true,
                    items: { "type": "string" }
                }
            },
            "actions": {
                "flipH": {},
                "flipV": {},
                "clear": {},
                "showMessage": {
                    input: {
                        type: "object",
                        description: "The message to show. May include the scrolling speed and text/background colours.",
                        properties: {
                            "textString": {
                                "type": "string"
                            },
                            "scrollSpeed": {
                                "type": "number",
                                "unit": "second",
                                "minimum": 0,
                                "maximum": 1,
                                "description": "seconds paused between letters."
                            },
                            "textColour": {
                                "type": "array",
                                "items": { "type": "integer", "minimum": 0, "maximum": 255 },
                                "minItems": 3,
                                "maxItems": 3,
                            },
                            "backColour": {
                                "type": "array",
                                "items": { "type": "integer", "minimum": 0, "maximum": 255 },
                                "minItems": 3,
                                "maxItems": 3,
                            }
                        },
                        required: ["textString", "scrollSpeed"]
                    }
                },
                "showLetter": {
                    input: {
                        type: "object",
                        description: "The letter to show. May include text/background colours.",
                        properties: {
                            "letter": {
                                "type": "string",
                                "maxLength": 1
                            },
                            "textColour": {
                                "type": "array",
                                "items": { "type": "integer", "minimum": 0, "maximum": 255 },
                                "minItems": 3,
                                "maxItems": 3,
                            },
                            "backColour": {
                                "type": "array",
                                "items": { "type": "integer", "minimum": 0, "maximum": 255 },
                                "minItems": 3,
                                "maxItems": 3,
                            }
                        },
                        required: ["letter"]
                    }
                },
                "flashMessage": {
                    input: {
                        type: "object",
                        description: "The message to show where only one letter is displayed at a time. May include the scroll speed and text/background colours.",
                        properties: {
                            "textString": {
                                "type": "string"
                            },
                            "speed": {
                                "type": "number"
                            },
                            "textColour": {
                                "type": "array",
                                "items": { "type": "integer", "minimum": 0, "maximum": 255 },
                                "minItems": 3,
                                "maxItems": 3,
                            },
                            "backColour": {
                                "type": "array",
                                "items": { "type": "integer", "minimum": 0, "maximum": 255 },
                                "minItems": 3,
                                "maxItems": 3,
                            }
                        },
                        required: ["textString"]
                    }
                },
                "setPixel": {
                    input: {
                        type: "object",
                        description: "Set the pixel in the given (x,y) coordinates to a given RGB color.",
                        properties: {
                            "x": {
                                "type": "integer",
                                "minimum": 0,
                                "maximum": 7
                            },
                            "y": {
                                "type": "integer",
                                "minimum": 0,
                                "maximum": 7
                            },
                            "r": {
                                "type": "integer",
                                "minimum": 0,
                                "maximum": 255
                            },
                            "g": {
                                "type": "integer",
                                "minimum": 0,
                                "maximum": 255
                            },
                            "b": {
                                "type": "integer",
                                "minimum": 0,
                                "maximum": 255
                            },
                        },
                        required: ["x", "y", "r", "g", "b"]
                    }
                }
            },
            "events": {
                "joystickPress": {
                    "data": {
                        "type": "string",
                        "enum": ["left", "right", "up", "down", "click"]
                    }
                },
                "joystickRelease": {
                    "data": {
                        "type": "string",
                        "enum": ["left", "right", "up", "down", "click"]
                    }
                },
                "joystickHold": {
                    "data": {
                        "type": "string",
                        "enum": ["left", "right", "up", "down", "click"]
                    }
                }
            }
        }).then((exposedThing) => {
            this.thing = exposedThing;
            this.td = exposedThing.getThingDescription();
            this.addPropertyHandlers();
            this.addActionHandlers();
            this.thing.expose();
            this.event_history = [];
            this.listen_to_joystick();
            sense.clear();
            this.pollSensors();
            if (tdDirectory) {
                this.register(tdDirectory);
            }
        });
    }
    pollSensors() {
        setInterval(() => {
            imu.getValue((err, data) => {
                if (err !== null) {
                    console.error("Could not read sensor data: ", err);
                    return;
                }
                this.thing.writeProperty("acceleration", data["accel"]);
                this.thing.writeProperty("gyro", data["gyro"]);
                this.thing.writeProperty("compass", data["compass"]);
                this.thing.writeProperty("humidity", data["humidity"]);
                this.thing.writeProperty("temperature", data["temperature"]);
                this.thing.writeProperty("pressure", data["pressure"]);
            });
        }, 1000);
    }
    addPropertyHandlers() {
        this.thing.setPropertyReadHandler("displayRotation", this.get_rotation);
        this.thing.setPropertyWriteHandler("displayRotation", (val, options) => {
            return new Promise((resolve, reject) => {
                if (!ajv.validate(this.td.properties.displayRotation, val)) {
                    reject(new Error("Invalid input"));
                }
                else {
                    resolve(this.set_rotation(val));
                }
            });
        });
        this.thing.setPropertyReadHandler("pixels", this.get_pixels);
        this.thing.setPropertyWriteHandler("pixels", (val, options) => {
            return new Promise((resolve, reject) => {
                if (!ajv.validate(this.td.properties.pixels, val)) {
                    reject(new Error("Invalid input"));
                }
                else {
                    resolve(this.set_pixels(val));
                }
            });
        });
        this.thing.setPropertyReadHandler("lowLight", this.get_lowLight);
        this.thing.setPropertyWriteHandler("lowLight", (val, options) => {
            return new Promise((resolve, reject) => {
                if (!ajv.validate(this.td.properties.lowLight, val)) {
                    reject(new Error("Invalid input"));
                }
                else {
                    resolve(this.set_lowlight(val));
                }
            });
        });
        this.thing.setPropertyReadHandler("humidity", () => { return this.get_imu_data("humidity"); });
        this.thing.setPropertyReadHandler("temperature", () => { return this.get_imu_data("temperature"); });
        this.thing.setPropertyReadHandler("pressure", () => { return this.get_imu_data("pressure"); });
        this.thing.setPropertyReadHandler("gyro", () => { return this.get_imu_data("gyro"); });
        this.thing.setPropertyReadHandler("acceleration", () => { return this.get_imu_data("accel"); });
        this.thing.setPropertyReadHandler("compass", () => { return this.get_imu_data("compass"); });
        this.thing.setPropertyReadHandler("eventHistory", () => { return this.get_events(); });
    }
    addActionHandlers() {
        this.thing.setActionHandler("flipH", () => { return this.flip_h(); });
        this.thing.setActionHandler("flipV", () => { return this.flip_v(); });
        this.thing.setActionHandler("showMessage", (message) => {
            return new Promise((resolve, reject) => {
                if (!ajv.validate(this.td.actions.showMessage.input, message)) {
                    reject(new Error("Invalid input"));
                }
                else {
                    resolve(this.show_message(message));
                }
            });
        });
        this.thing.setActionHandler("showLetter", (message) => {
            return new Promise((resolve, reject) => {
                if (!ajv.validate(this.td.actions.showLetter.input, message)) {
                    reject(new Error("Invalid input"));
                }
                else {
                    resolve(this.show_letter(message));
                }
            });
        });
        this.thing.setActionHandler("flashMessage", (message) => {
            return new Promise((resolve, reject) => {
                if (!ajv.validate(this.td.actions.flashMessage.input, message)) {
                    reject(new Error("Invalid input"));
                }
                else {
                    resolve(this.flash_message(message));
                }
            });
        });
        this.thing.setActionHandler("setPixel", (message) => {
            return new Promise((resolve, reject) => {
                if (!ajv.validate(this.td.actions.setPixel.input, message)) {
                    reject(new Error("Invalid input"));
                }
                else {
                    resolve(this.set_pixel(message));
                }
            });
        });
        this.thing.setActionHandler("clear", () => { return this.clear(); });
    }
    register(directory) {
        console.log("Registering TD in directory: " + directory);
        request.post(directory, { json: this.thing.getThingDescription() }, (error, response, body) => {
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
    // ------------------------  Properties  ------------------------
    set_rotation(angle) {
        return new Promise((resolve, reject) => {
            if ([0, 90, 180, 270].includes(angle)) {
                sense.setRotation(angle, (err, data) => {
                    if (err) {
                        console.error("Could not perform action setRotation: " + err);
                        reject(err);
                    }
                    else {
                        resolve();
                    }
                });
            }
            else {
                reject("Angle must be 0, 90, 180 or 270.");
            }
        });
    }
    get_rotation() {
        return new Promise((resolve, reject) => {
            resolve(sense.rotation);
        });
    }
    set_lowlight(value) {
        return new Promise((resolve, reject) => {
            if (typeof (value) === "boolean") {
                sense.lowLight = value;
                resolve();
            }
            else {
                reject("Value must be a boolean.");
            }
        });
    }
    get_lowLight() {
        return new Promise((resolve, reject) => {
            resolve(sense.lowLight);
        });
    }
    set_pixels(pixel_array) {
        return new Promise((resolve, reject) => {
            sense.setPixels(pixel_array, (err, data) => {
                if (err) {
                    console.error("Could not perform action setPixels: " + err);
                    reject(err);
                }
                else {
                    resolve();
                }
            });
        });
    }
    get_pixels() {
        return new Promise((resolve, reject) => {
            sense.getPixels((err, data) => {
                if (err) {
                    console.error("Could not perform action getPixels: " + err);
                    reject(err);
                }
                else {
                    resolve(data);
                }
            });
        });
    }
    get_imu_data(property) {
        return new Promise((resolve, reject) => {
            imu.getValue((err, data) => {
                if (err !== null) {
                    console.error("Could not read sensor data: ", err);
                    reject("Could not read sensor data: " + String(err));
                    return;
                }
                if (["accel", "gyro", "compass", "humidity", "temperature", "pressure"].includes(property)) {
                    resolve(data[property]);
                }
                else {
                    reject("Property '" + property + "' does not exist.");
                }
            });
        });
    }
    get_events() {
        return new Promise((resolve, reject) => {
            resolve(this.event_history);
        });
    }
    // ------------------------  Actions  ------------------------
    flip_h() {
        return new Promise((resolve, reject) => {
            sense.flipH((err, data) => {
                if (err) {
                    console.error("Could not perform action FlipH: " + err);
                    reject(err);
                }
                else {
                    resolve();
                }
            });
        });
    }
    flip_v() {
        return new Promise((resolve, reject) => {
            sense.flipV((err, data) => {
                if (err) {
                    console.error("Could not perform action flipV: " + err);
                    reject(err);
                }
                else {
                    resolve();
                }
            });
        });
    }
    show_message(message) {
        return new Promise((resolve, reject) => {
            sense.showMessage(message.textString, message.scrollSpeed, message.textColour, message.backColour, (err, data) => {
                if (err) {
                    console.error("Could not perform action showMessage: " + err);
                    reject(err);
                }
                else {
                    resolve();
                }
            });
        });
    }
    flash_message(message) {
        return new Promise((resolve, reject) => {
            sense.flashMessage(message.textString, message.speed, message.textColour, message.backColour, (err, data) => {
                if (err) {
                    console.error("Could not perform action flashMessage: " + err);
                    reject(err);
                }
                else {
                    resolve();
                }
            });
        });
    }
    show_letter(message) {
        return new Promise((resolve, reject) => {
            sense.showLetter(message.letter, message.text_colour, message.back_colour, (err, data) => {
                if (err) {
                    console.error("Could not perform action showLetter: " + err);
                    reject(err);
                }
                else {
                    resolve();
                }
            });
        });
    }
    set_pixel(pixel) {
        return new Promise((resolve, reject) => {
            sense.setPixel(pixel.x, pixel.y, pixel.r, pixel.g, pixel.b, (err, data) => {
                if (err) {
                    console.error("Could not perform action setPixel: " + err);
                    reject(err);
                }
                else {
                    resolve();
                }
            });
        });
    }
    clear() {
        return new Promise((resolve, reject) => {
            sense.clear((err, data) => {
                if (err) {
                    console.error("Could not perform action clear: " + err);
                    reject(err);
                }
                else {
                    resolve();
                }
            });
        });
    }
    // ------------------------  Events  ------------------------
    listen_to_joystick() {
        sense_joystick.getJoystick()
            .then((joystick) => {
            joystick.on("press", direction => {
                console.log("Joystick pressed in direction: " + direction);
                this.thing.emitEvent("joystickPress", direction);
                this.event_history.push("Press: " + direction);
            });
            joystick.on("release", direction => {
                console.log("Joystick released in direction: " + direction);
                this.thing.emitEvent("joystickRelease", direction);
                this.event_history.push("Release: " + direction);
            });
            joystick.on("hold", direction => {
                console.log("The joystick is being held in direction: " + direction);
                this.thing.emitEvent("joystickHold", direction);
                this.event_history.push("Hold: " + direction);
            });
        })
            .catch((err) => { console.error("Could not get joystick handler: " + err); });
    }
}
exports.WotSenseHat = WotSenseHat;
//# sourceMappingURL=sensehat.js.map