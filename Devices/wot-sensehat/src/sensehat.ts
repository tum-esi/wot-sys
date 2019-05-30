import * as WoT from "wot-typescript-definitions"
import { networkInterfaces } from "os"

const request = require('request');
const sense = require("sense-hat-led");
const nodeimu = require("nodeimu");
const imu = new nodeimu.IMU();
const sense_joystick = require("sense-joystick")


export class WotSenseHat {
    public thing: WoT.ExposedThing;
    public factory: WoT.WoTFactory;
    private event_history: string[];

    constructor(thingFactory: WoT.WoTFactory, tdDirectory?: string) {
        this.factory = thingFactory;
        this.thing = thingFactory.produce(`{
            "@context": [
                "https://www.w3.org/2019/wot/td/v1",
                { "@language" : "en" }],
            "@type": "Thing",
            "title" : "SenseHat",
            "description" : "Raspberry Pi SenseHat",
            "securityDefinitions": { "nosec_sc": { "scheme": "nosec" }},
            "security": "nosec_sc"
        }`);
        while (!networkInterfaces().wlan0) { }  // Wait for the network to get an IP address
        this.thing.id = "de:tum:ei:esi:sensehat:" + networkInterfaces().wlan0[0].address;
        this.add_properties();
        this.add_actions();
        this.add_events();
        this.event_history = [];
        this.listen_to_joystick();
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

// ------------------------  Properties  ------------------------

    private set_rotation(angle: number) {
        return new Promise((resolve, reject) => {
            if ([0,90,180,270].includes(angle)) {
                sense.setRotation(angle, (err, data) => {
                    if (err) {
                        console.error("Could not perform action setRotation: " + err)
                        reject(err)
                    } else {
                        resolve()
                    }
                })
            } else {
                reject("Angle must be 0, 90, 180 or 270.")
            }
        });
    }

    private get_rotation() {
        return new Promise((resolve, reject) => {
                resolve(sense.rotation);
        });
    }

    private set_lowlight(value: boolean) {
        return new Promise((resolve, reject) => {
            if (typeof(value) === "boolean") {
                sense.lowLight = value;
                resolve();
            } else {
                reject("Value must be a boolean.")
            }
        });
    }

    private get_lowLight() {
        return new Promise((resolve, reject) => {
            resolve(sense.lowLight);
        });
    }

    private set_pixels(pixel_array: number[][]) {
        return new Promise((resolve, reject) => {
            sense.setPixels(pixel_array, (err, data) => {
                if (err) {
                    console.error("Could not perform action setPixels: " + err)
                    reject(err)
                } else {
                    resolve()
                }
            });
        });
    }

    private get_pixels() {
        return new Promise((resolve, reject) => {
            sense.getPixels((err, data) => {
                if (err) {
                    console.error("Could not perform action getPixels: " + err)
                    reject(err)
                } else {
                    resolve(data)
                }
            });
        });
    }

    private get_imu_data(property: string) {
        return new Promise((resolve, reject) => {
            imu.getValue((err, data) => {
                if (err !== null) {
                  console.error("Could not read sensor data: ", err);
                  reject("Could not read sensor data: " + String(err));
                  return;
                }
                if (["accel", "gyro", "compass", "humidity", "temperature", "pressure"].includes(property)) {
                    resolve(data[property])
                } else {
                    reject("Property '" + property + "' does not exist.")
                }
            });
        });
    }

    private get_events() {
        return new Promise((resolve, reject) => {
            resolve(this.event_history)
        });
    }

    private add_properties() {
        this.thing.addProperty(
            "displayRotation",
            {
                type: "integer", 
                readOnly: false,
                writeOnly: false,
                description: "The rotation of the display. 0 is with the HDMI port facing down.", 
                enum: [0, 90, 180, 270]
            }
        );
        this.thing.setPropertyReadHandler("displayRotation", this.get_rotation);
        this.thing.setPropertyWriteHandler("displayRotation", this.set_rotation);

        this.thing.addProperty(
            "pixels",
            {
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
            }
        );
        this.thing.setPropertyReadHandler("pixels", this.get_pixels);
        this.thing.setPropertyWriteHandler("pixels", this.set_pixels);

        this.thing.addProperty(
            "lowLight",
            {
                type: "boolean", 
                readOnly: false,
                writeOnly: false,
                description: "The LED Matrix low light mode.", 
            }
        );
        this.thing.setPropertyReadHandler("lowLight", this.get_lowLight);
        this.thing.setPropertyWriteHandler("lowLight", this.set_lowlight);

        this.thing.addProperty(
            "humidity",
            {
                type: "number", 
                description: "The percentage of relative humidity.",
                readOnly: true,
            }
        );
        this.thing.setPropertyReadHandler("humidity", () => { return this.get_imu_data("humidity"); });

        this.thing.addProperty(
            "temperature",
            {
                type: "number", 
                description: "The current temperature in degrees Celsius.",
                readOnly: true,
            }
        );
        this.thing.setPropertyReadHandler("temperature", () => { return this.get_imu_data("temperature"); });

        this.thing.addProperty(
            "pressure",
            {
                type: "number", 
                description: "The current pressure in Millibars.",
                readOnly: true,
            }
        );
        this.thing.setPropertyReadHandler("pressure", () => { return this.get_imu_data("pressure"); });

        this.thing.addProperty(
            "gyro",
            {
                type: "object", 
                description: "Get 3 Floats representing the rotational intensity of each axis in rads/second.",
                readOnly: true,
                properties: {
                    "x": { "type": "number" }, 
                    "y": { "type": "number" },
                    "z": { "type": "number" }
                }
            }
        );
        this.thing.setPropertyReadHandler("gyro", () => { return this.get_imu_data("gyro"); });

        this.thing.addProperty(
            "accel",
            {
                type: "object", 
                description: "Get 3 Floats representing the acceleration intensity of each axis in Gs",
                readOnly: true,
                properties: {
                    "x": { "type": "number" }, 
                    "y": { "type": "number" },
                    "z": { "type": "number" }
                }
            }
        );
        this.thing.setPropertyReadHandler("accel", () => { return this.get_imu_data("accel"); });

        this.thing.addProperty(
            "compass",
            {
                type: "object", 
                description: "Get 3 Floats representing the magnetic intensity of the axis in microteslas.",
                readOnly: true,
                properties: {
                    "x": { "type": "number" }, 
                    "y": { "type": "number" },
                    "z": { "type": "number" }
                }
            }
        );
        this.thing.setPropertyReadHandler("compass", () => { return this.get_imu_data("compass"); });

        this.thing.addProperty(
            "eventHistory",
            {
                type: "array", 
                description: "An array containing the complete event history.",
                readOnly: true,
                items: { "type": "string" }
            }
        );
        this.thing.setPropertyReadHandler("eventHistory", () => { return this.get_events(); });
    }

// ------------------------  Actions  ------------------------

    private flip_h() {
        return new Promise((resolve, reject) => {
            sense.flipH((err, data) => {
                if (err) {
                    console.error("Could not perform action FlipH: " + err)
                    reject(err)
                } else {
                    resolve()
                }
            });
        });
    }

    private flip_v() {
        return new Promise((resolve, reject) => {
            sense.flipV((err, data) => {
                if (err) {
                    console.error("Could not perform action flipV: " + err)
                    reject(err)
                } else {
                    resolve()
                }
            });
        });
    }

    private show_message(message: { textString: string, scrollSpeed?: number, textColour?: number[], backColour?: number[] }) {
        return new Promise((resolve, reject) => {
            sense.showMessage(message.textString, message.scrollSpeed, message.textColour, message.backColour, (err, data) => {
                if (err) {
                    console.error("Could not perform action showMessage: " + err)
                    reject(err)
                } else {
                    resolve()
                }
            })
        });
    }

    private flash_message(message: { textString: string, speed?: number, textColour?: number[], backColour?: number[] }) {
        return new Promise((resolve, reject) => {
            sense.flashMessage(message.textString, message.speed, message.textColour, message.backColour, (err, data) => {
                if (err) {
                    console.error("Could not perform action flashMessage: " + err)
                    reject(err)
                } else {
                    resolve()
                }
            })
        });
    }

    private show_letter(message: { s: string, text_colour?: number[], back_colour?: number[] }) {
        return new Promise((resolve, reject) => {
            sense.showLetter(message.s, message.text_colour, message.back_colour, (err, data) => {
                if (err) {
                    console.error("Could not perform action showLetter: " + err)
                    reject(err)
                } else {
                    resolve()
                }
            })
        });
    }

    private set_pixel(pixel: {x: number, y: number, r: number, g: number, b: number}) {
        return new Promise((resolve, reject) => {
            sense.setPixel(pixel.x, pixel.y, pixel.r, pixel.g, pixel.b, (err, data) => {
                if (err) {
                    console.error("Could not perform action setPixel: " + err)
                    reject(err)
                } else {
                    resolve()
                }
            })
        });
    }

    private clear() {
        return new Promise((resolve, reject) => {
            sense.clear((err, data) => {
                if (err) {
                    console.error("Could not perform action clear: " + err)
                    reject(err)
                } else {
                    resolve()
                }
            })
        });
    }

    private add_actions() {
        this.thing.addAction(
            "flipH",
            { },
            () => { return this.flip_h(); }
        );
        this.thing.addAction(
            "flipV",
            { },
            () => { return this.flip_v(); }
        );
        this.thing.addAction(
            "showMessage",
            {
                input: {
                    type: "object",
                    description: "The message to show. May include the scrolling speed and text/background colours.",
                    properties: {
                        "textString": {
                            "type": "string"
                        }, 
                        "scrollSpeed": {
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
            (message) => { return this.show_message(message); }
        );
        this.thing.addAction(
            "showLetter",
            {
                input: {
                    type: "object",
                    description: "The letter to show. May include text/background colours.",
                    properties: {
                        "s": {
                            "type": "string"
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
                    required: ["s"]
                }
            },
            (message) => { return this.show_letter(message); }
        );
        this.thing.addAction(
            "flashMessage",
            {
                input: {
                    type: "object",
                    description: "The message to show. May include the flashing speed and text/background colours.",
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
            (message) => { return this.flash_message(message); }
        );
        this.thing.addAction(
            "setPixel",
            {
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
                    required: ["textString"]
                }
            },
            (message) => { return this.set_pixel(message); }
        );
        this.thing.addAction(
            "clear",
            { },
            () => { return this.clear(); }
        );
    }

    // ------------------------  Events  ------------------------

    private listen_to_joystick() {
        sense_joystick.getJoystick()
        .then((joystick) => {
            joystick.on("press", direction => {
                console.log("Joystick pressed in direction: " + direction);
                this.thing.events["joystickPress"].emit(direction);
                this.event_history.push("Press: " + direction)
            });
            joystick.on("release", direction => {
                console.log("Joystick released in direction: " + direction);
                this.thing.events["joystickRelease"].emit(direction);
                this.event_history.push("Release: " + direction)
            });
            joystick.on("hold", direction => {
                console.log("The joystick is being held in direction: " + direction);
                this.thing.events["joystickHold"].emit(direction);
                this.event_history.push("Hold: " + direction)
            });
        })
        .catch((err) => { console.error("Could not get joystick handler: " + err); });
    }

    private add_events() {
        this.thing.addEvent(
            "joystickPress",
            {
                "type": "string", 
                "enum": ["left", "right", "up", "down", "click"]
            }
        );
        this.thing.addEvent(
            "joystickRelease",
            {
                "type": "string", 
                "enum": ["left", "right", "up", "down", "click"]
            }
        );
        this.thing.addEvent(
            "joystickHold",
            {
                "type": "string", 
                "enum": ["left", "right", "up", "down", "click"]
            }
        )
    }
}