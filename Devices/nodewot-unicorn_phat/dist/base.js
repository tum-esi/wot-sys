"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.WotDevice = void 0;
var request = require('request');
const Ajv = require('ajv');
var ajv = new Ajv();
var ws281x = require('rpi-ws281x');
class WotDevice {
    constructor(WoT, tdDirectory) {
        //create WotDevice as a server
        this.WoT = WoT;
        this.WoT.produce(
        //fill in the empty quotation marks
        {
            "@context": [
                "https://www.w3.org/2019/wot/td/v1",
                { "@language": "en" }
            ],
            "@type": "",
            id: "urn:dev:ops:32473-UnicornpHAT-001",
            title: "UnicornPHAT",
            description: "LED matrix",
            securityDefinitions: {
                "nosec_sc": {
                    "scheme": "nosec_sc"
                }
            },
            security: "nosec_sc",
            properties: {
                pixels: {
                    title: "pixels",
                    description: "Array of pixels with their RGB colors",
                    type: "array",
                    items: {
                        type: "integer"
                    },
                    observable: true,
                    readOnly: false,
                    minItems: 32,
                    maxItems: 32
                }
            },
            actions: {
                fillColor: {
                    input: {
                        "oneOf": [
                            {
                                "type": "string",
                                "enum": ["red", "green", "blue"],
                                "description": "fully saturated one color as string"
                            },
                            {
                                type: "array",
                                items: {
                                    "type": "integer",
                                    "maximum": 255,
                                    "minimum": 0
                                },
                                minItems: 3,
                                maxItems: 3,
                                "description": "[r,g,b] input"
                            }
                        ]
                    }
                },
                fillRandom: {}
            }
        }).then((exposedThing) => {
            this.thing = exposedThing;
            this.td = exposedThing.getThingDescription();
            this.addPropertyHandlers();
            this.addActionHandlers();
            this.thing.expose();
            if (tdDirectory) {
                this.register(tdDirectory);
            }
            //configuring the phat
            this.config = {};
            // Number of leds in my strip
            this.config.leds = 32;
            // Use DMA 10 (default 10)
            this.config.dma = 10;
            // Set full brightness, a value from 0 to 255 (default 255)
            this.config.brightness = 255;
            // Set the GPIO number to communicate with the Neopixel strip (default 18)
            this.config.gpio = 18;
            // The RGB sequence may vary on some strips. Valid values
            // are "rgb", "rbg", "grb", "gbr", "bgr", "brg".
            // Default is "rgb".
            // RGBW strips are not currently supported.
            this.config.type = 'grb';
            // Configure ws281x
            ws281x.configure(this.config);
            this.pixels = new Uint32Array(this.config.leds);
            // Create a fill color with red/green/blue for the initial screen
            var red = 100, green = 100, blue = 100;
            var color = (red << 16) | (green << 8) | blue;
            for (var i = 0; i < this.config.leds; i++)
                this.pixels[i] = color;
            // Render to strip
            this.thing.writeProperty("pixels", this.convertToArray(this.pixels));
        });
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
    // -----------------------  ADD P, A & E  -----------------------
    addPropertyHandlers() {
        this.thing.setPropertyWriteHandler("pixels", (val, options) => {
            return new Promise((resolve, reject) => {
                if (!ajv.validate(this.td.properties.pixels, val)) {
                    reject(new Error("Invalid input"));
                }
                else {
                    this.pixels = new Uint32Array(val);
                    ws281x.render(this.pixels);
                    resolve(this.convertToArray(this.pixels));
                }
            });
        });
    }
    addActionHandlers() {
        this.thing.setActionHandler("fillColor", (message) => {
            return new Promise((resolve, reject) => {
                if (!ajv.validate(this.td.actions.fillColor.input, message)) {
                    reject(new Error("Invalid input"));
                }
                else {
                    if (message == "red") {
                        this.fillColor(255, 0, 0);
                        resolve();
                    }
                    else if (message == "green") {
                        this.fillColor(0, 255, 0);
                        resolve();
                    }
                    else if (message == "blue") {
                        this.fillColor(0, 0, 255);
                        resolve();
                    }
                    else { //then it is the array
                        this.fillColor(message[0], message[1], message[2]);
                        resolve();
                    }
                }
            });
        });
        this.thing.setActionHandler("fillRandom", (message) => {
            return new Promise((resolve, reject) => {
                this.fillRandom();
                resolve();
            });
        });
    }
    // HW related functions
    fillColor(r, g, b) {
        var red = r, green = g, blue = b;
        var color = (red << 16) | (green << 8) | blue;
        for (var i = 0; i < this.config.leds; i++)
            this.pixels[i] = color;
        this.thing.writeProperty("pixels", this.convertToArray(this.pixels));
    }
    fillRandom() {
        for (var i = 0; i < this.config.leds; i++)
            this.pixels[i] = this.getRandomColor();
        this.thing.writeProperty("pixels", this.convertToArray(this.pixels));
    }
    convertToArray(uintArray) {
        var s = JSON.stringify(uintArray);
        var a = JSON.parse(s);
        var arr = [];
        for (var i in Object.getOwnPropertyNames(a)) {
            arr[i] = a[i];
        }
        return (arr);
    }
    getRandomColor() {
        var min = 0;
        var max = 255;
        var red = Math.floor(Math.random() * (max - min + 1)) + min;
        var green = Math.floor(Math.random() * (max - min + 1)) + min;
        var blue = Math.floor(Math.random() * (max - min + 1)) + min;
        var color = (red << 16) | (green << 8) | blue;
        return color;
    }
}
exports.WotDevice = WotDevice;
//# sourceMappingURL=base.js.map