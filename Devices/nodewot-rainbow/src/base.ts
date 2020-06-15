import * as WoT from "wot-typescript-definitions"

var request = require('request');
const Ajv = require('ajv');
var ajv = new Ajv();


// For the seven segment displays
var FourteenSegment = require('ht16k33-fourteensegment-display');
const display = new FourteenSegment(0x70, 1);

// For the temperature and pressure sensors
const BME280 = require('bme280-sensor');
const bmeOptions = {
  i2cBusNo:1,
  i2cAddress:BME280.BME280_DEFAULT_I2C_ADDRESS()
}
const bme280 = new BME280(bmeOptions);

// For the touch buttons and their associated LEDs
const gpio = require('rpi-gpio');
gpio.setMode(gpio.MODE_BCM);
const channels = {
    touchA: 21,
    touchB: 20,
    touchC: 16,
    ledA: 6,
    ledB: 19,
    ledC: 26
}

// For the Rainbow LEDs
const spi = require('spi-device');
const spiBus = spi.openSync(0,0);
const apa102 = new Apa102spi(7);

    function turnOff() {
        for (let i = 0; i < 7; i++) {
            apa102.setLedColor(i, 0, 0, 0, 0);
        }
        apa102.sendLeds();
    }

    function Apa102spi (stringLength) {
        this.bufferLength = stringLength * 4;
        this.writeBuffer = Buffer.alloc(this.bufferLength);
        this.bufferLength += 9;
        this.writeBuffer = Buffer.concat([Buffer.alloc(4), this.writeBuffer, Buffer.alloc(5)], this.bufferLength);
    }

    // Must be called to send the buffered values
    Apa102spi.prototype.sendLeds = function () {
        const message = [{
            sendBuffer: this.writeBuffer,
            byteLength: this.bufferLength
        }];

        spiBus.transferSync(message);
    };

    Apa102spi.prototype.setLedColor = function (n, brightness, r, g, b) {
        n *= 4;
        n += 4;
        this.writeBuffer[n] = brightness | 0b11100000;
        this.writeBuffer[n + 1] = b;
        this.writeBuffer[n + 2] = g;
        this.writeBuffer[n + 3] = r;
    };
    
var leds = {
    "0":{
      "brightness":5,
      "colour":[255,255,255]
    },
    "1":{
      "brightness":5,
      "colour":[255,255,255]
    },
    "2":{
      "brightness":5,
      "colour":[255,255,255]
    },
    "3":{
      "brightness":5,
      "colour":[255,255,255]
    },
    "4":{
      "brightness":5,
      "colour":[255,255,255]
    },
    "5":{
      "brightness":5,
      "colour":[255,255,255]
    },
    "6":{
      "brightness":5,
      "colour":[255,255,255]
    }
}
export class WotDevice {
    public thing: WoT.ExposedThing;
    public WoT: WoT.WoT;
    public td: any;
	
    constructor(WoT: WoT.WoT, tdDirectory?: string) {
        //create WotDevice as a server
        this.WoT = WoT;
        this.WoT.produce(
{
  "@context": [
      "https://www.w3.org/2019/wot/td/v1",
      { "@language" : "en" }],
  "@type": "",
  id : "urn:dev:ops:32473-rainbowhat-001",
  title : "RainbowHAT1",
  description : "HAT with seven segment displays, temperature and pressure sensors, touch buttons and LEDs",
  securityDefinitions: { 
      "nosec_sc": { 
          "scheme": "nosec_sc" 
      }
  },
  security: "nosec_sc",
  properties:{
    leds:{
      title:"LEDs",
      description: "7 LEDs with their RGB colors. 0 corresponds to the rightmost LED. They can be individually set",
  "type": "object",
  "propertyNames": {
    "type": "string",
    "enum":["0","1","2","3","4","5","6"]
  },
  "additionalProperties": {
    "type": "object",
    "properties": {
      "brightness": {
        "type": "integer",
        "maximum": 15,
        "minimum": 0
      },
      "colour": {
        "type": "array",
        "maxItems": 3,
        "minItems": 3,
        "items": {
          "type": "integer",
          "maximum": 255,
          "minimum": 0
        }
      }
    },
    "required": ["brightness","colour"]
  },
     readOnly: false
    },
    pressure:{
      title:"pressure",
      description: "Pressure in hPa",
      type: "number",
      unit: "hPa",
      observable: true,
      readOnly: true
    },
    temperature:{
      title:"temperature",
      description: "Temperature in Celcius",
      type: "number",
      unit: "C",
      observable: true,
      readOnly: true
    }
  },
  actions:{
    writeDisplay:{
      input:{
	"type":"string",
	"maxLength":4,
	"description":"A string from left to right on the seven segment display. Capitals look better"
      }
    },
    clearDisplay:{
      "description":"Clears the seven segment displays"
    },
    clearLEDs:{
      "description":"Turns off the 7 LEDs"
    },
    makeRainbow:{
      "description":"Changes the colors of the LEDs to the official Rainbow colors!"
    }
  },
  events:{
    buttonA:{
      description:"Press (true) or release(false) of a button. An LED also lights up on the HAT",
      data:{
	type:"boolean"
      }
    },
    buttonB:{
      description:"Press (true) or release(false) of a button. An LED also lights up on the HAT",
      data:{
	type:"boolean"
      }
    },
    buttonC:{
      description:"Press (true) or release(false) of a button. An LED also lights up on the HAT",
      data:{
	type:"boolean"
      }
    }
  }
}
        ).then((exposedThing)=>{
	  this.thing = exposedThing;
	  this.td = exposedThing.getThingDescription();
	  this.addPropertyHandlers();
	  this.addActionHandlers();
	  this.thing.expose();
	  bme280.init().then(()=>{
	    this.pollSensors();
	  });
	  
	  this.setupButtons();
	  
	  this.thing.writeProperty("leds",leds);

	  if (tdDirectory) { this.register(tdDirectory); }
        });
        
    }
    public register(directory: string) {
        console.log("Registering TD in directory: " + directory)
        request.post(directory, {json: this.thing.getThingDescription()}, (error, response, body) => {
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
    
    private pollSensors(){
        setInterval(()=>{
            bme280.readSensorData().then((data) => {
                
                this.thing.writeProperty("temperature",data.temperature_C)
                this.thing.writeProperty("pressure",data.pressure_hPa)

            });
        },1000)
    }
    
    private setupButtons(){
      var that = this;
      gpio.on('change', function(channel,value){
	if (channel == channels.touchA){
	  gpio.write(channels.ledA, !value);
	  that.thing.emitEvent("buttonA",!value);
	}
	if (channel == channels.touchB){
	  gpio.write(channels.ledB, !value);
	  that.thing.emitEvent("buttonB",!value)
	}
	if (channel == channels.touchC){
	  gpio.write(channels.ledC, !value);
	  that.thing.emitEvent("buttonC",!value)
	}
      });
      gpio.setup(channels.touchA, gpio.DIR_IN, gpio.EDGE_BOTH);
      gpio.setup(channels.touchB, gpio.DIR_IN, gpio.EDGE_BOTH);
      gpio.setup(channels.touchC, gpio.DIR_IN, gpio.EDGE_BOTH);
      gpio.setup(channels.ledA, gpio.DIR_OUT);
      gpio.setup(channels.ledB, gpio.DIR_OUT);
      gpio.setup(channels.ledC, gpio.DIR_OUT);
    }
    

    // ------------------------  Properties  ------------------------


    // -----------------------  ADD P, A & E  -----------------------

    private addPropertyHandlers() {
      this.thing.setPropertyWriteHandler("leds", 
        (val ,options) => { 
            return new Promise((resolve, reject) => {
                if (!ajv.validate(this.td.properties.leds, val)) {
		  reject(new Error ("Invalid input: "+ajv.errorsText()));
                }
                else {
		  for ( var property in val) {
		    var brigCol = val[property];
		    apa102.setLedColor(property,brigCol.brightness , brigCol.colour[0], brigCol.colour[1], brigCol.colour[2]);
		    leds[property] = brigCol;
		  }
		  apa102.sendLeds();
		  resolve(leds);
                }
            });
        });
    }
    
    private addActionHandlers() {
      this.thing.setActionHandler(
            "writeDisplay",
            (message) => { 
                return new Promise((resolve, reject) => {
                    if (!ajv.validate(this.td.actions.writeDisplay.input, message)) {
                        reject(new Error ("Invalid input"));
                    }
                    else {
			display.writeString(message);
			resolve();
                    }
                });
                
            }
        );

      this.thing.setActionHandler(
            "clearDisplay",
            (message) => {
                return new Promise((resolve, reject) => {
		  display.clear();
		  resolve();
                });
            }
        );
      this.thing.setActionHandler(
            "clearLEDs",
            (message) => {
                return new Promise((resolve, reject) => {
		  turnOff();
		  resolve();
                });
            }
        );
	
	this.thing.setActionHandler(
	  "makeRainbow",
	  (message) => {
	      return new Promise((resolve, reject) => {
		leds = {
		  "0":{
		    "brightness":10,
		    "colour":[139,0,255]
		  },
		  "1":{
		    "brightness":10,
		    "colour":[46,43,95]
		  },
		  "2":{
		    "brightness":10,
		    "colour":[0,0,255]
		  },
		  "3":{
		    "brightness":10,
		    "colour":[0,255,0]
		  },
		  "4":{
		    "brightness":10,
		    "colour":[255,255,0]
		  },
		  "5":{
		    "brightness":10,
		    "colour":[255,127,0]
		  },
		  "6":{
		    "brightness":10,
		    "colour":[255,0,0]
		  }
		}
		this.thing.writeProperty("leds",leds);
		resolve();
	      });
	  }
	);
	
	
    }
    
}
