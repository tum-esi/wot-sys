import * as WoT from "wot-typescript-definitions"

var request = require('request');
const Ajv = require('ajv');
var ajv = new Ajv();

var FourteenSegment = require('ht16k33-fourteensegment-display');
const display = new FourteenSegment(0x70, 1);

const BME280 = require('bme280-sensor');
const bmeOptions = {
  i2cBusNo:1,
  i2cAddress:BME280.BME280_DEFAULT_I2C_ADDRESS()
}
const bme280 = new BME280(bmeOptions);

const gpio = require('rpi-gpio');
gpio.setMode(gpio.MODE_BCM);
const channels = {
    A: 21,
    B: 20,
    C: 16
}

export class WotDevice {
    public thing: WoT.ExposedThing;
    public WoT: WoT.WoT;
    public td: any;
	
    constructor(WoT: WoT.WoT, tdDirectory?: string) {
        //create WotDevice as a server
        this.WoT = WoT;
        this.WoT.produce(
            //fill in the empty quotation marks
{
  "@context": [
      "https://www.w3.org/2019/wot/td/v1",
      { "@language" : "en" }],
  "@type": "",
  id : "urn:dev:ops:32473-rainbowhat-001",
  title : "RainbowHAT",
  description : "HAT with seven segment displays, temperature and pressure sensors, touch buttons and LEDs",
  securityDefinitions: { 
      "nosec_sc": { 
          "scheme": "nosec_sc" 
      }
  },
  security: "nosec_sc",
  properties:{
    pixels:{
      title:"pixels",
      description: "Array of pixels with their RGB colors",
      type: "array",
      items:{
	type:"integer"
      },
      observable: true,
      readOnly: false,
      minItems:32,
      maxItems:32
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
	"description":"A string from left to right on the seven segment display"
      }
    },
    clearDisplay:{
    }
  },
  events:{
    buttonA:{
      description:"Press (true) or release(false) of a button",
      data:{
	type:"boolean"
      }
    },
    buttonB:{
      description:"Press (true) or release(false) of a button",
      data:{
	type:"boolean"
      }
    },
    buttonC:{
      description:"Press (true) or release(false) of a button",
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
	if (channel == channels.A){
	  that.thing.emitEvent("buttonA",!value)
	}
	if (channel == channels.B){
	  that.thing.emitEvent("buttonB",!value)
	}
	if (channel == channels.C){
	  that.thing.emitEvent("buttonC",!value)
	}
      });
      gpio.setup(channels.A, gpio.DIR_IN, gpio.EDGE_BOTH);
      gpio.setup(channels.B, gpio.DIR_IN, gpio.EDGE_BOTH);
      gpio.setup(channels.C, gpio.DIR_IN, gpio.EDGE_BOTH);
    }
    

    // ------------------------  Properties  ------------------------


    // -----------------------  ADD P, A & E  -----------------------

    private addPropertyHandlers() {
        
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
    }
}
