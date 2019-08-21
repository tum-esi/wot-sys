import * as WoT from "wot-typescript-definitions"
import { networkInterfaces } from "os"

var request = require('request')
var north = -1

const { spawn } = require('child_process')

export class WoTEnviroPHatPi {
	public thing: WoT.ExposedThing;
	public factory: WoT.WoTFactory;

	constructor(thingFactory: WoT.WoTFactory, tdDirectory?: string){
		this.factory = thingFactory;
		this.thing = thingFactory.produce(`{
			"@context": [
				"https://www.w3.org/2019/wot/td/v1",
				{ "@language" : "en" }],
			"@type": "Thing",
			"title": "EnviroPHat",
			"description": "Enviro Sensorhat For Pi Zero",
			"securityDefinitions": {"nosec_sc":{"scheme":"nosec"}},
			"security": "nosec_sc"
		}`);
		while (!networkInterfaces().wlan0) {}
		this.thing.id = "de:tum:ei:esi:envirophatpi:" + networkInterfaces().wlan0[0].address;
		this.add_properties();
		this.add_actions();
		this.thing.expose();
		if (tdDirectory) { this.register(tdDirectory); }
	}

	public register(directory: string){
		console.log("Registering TD in directory: " + directory)
		request.post(directory, {json: this.get_nonld_td()}, (error, response, body)=> {
			if (!error && response.statusCode < 300) {
				console.log("TD registered!");			
			} else {
				console.debug(error);
				console.debug(response);
				console.warn("Failed to register TD. Will try again in 10 seconds...");
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

//-------------------------------- Properties -----------------------------------------	
	private get_temp(){
	    return new Promise((resolve, reject) => {
		const process = spawn('python', ['./lib/enviro_phat.py', 'read_temp']);
                process.stdout.on('data', (data) => {
		    resolve(parseFloat(data.toString()));
	        });
	    });
	}

	private get_pressure(){
	    return new Promise((resolve, reject) => {
		const process = spawn('python', ['./lib/enviro_phat.py', 'read_pressure']);
		process.stdout.on('data', (data) => {
		    resolve(parseFloat(data.toString()));
	        });
	    });
	}

	private get_motion(){
	    return new Promise((resolve, reject) => {
		const process = spawn('python', ['./lib/enviro_phat.py', 'read_motion']);
                process.stdout.on('data', (data) => {
		    resolve(data.toString());
	        });
	    });
	}	

	private get_heading(){
	    //var that = this;
	    return new Promise((resolve, reject) => {
		//that.thing.properties["north"].read().then((north) => {
		    const process = spawn('python', ['./lib/enviro_phat.py', 'read_heading']);
		    process.stdout.on('data', (data) => {
		        if(north>=0){
			    var result = (parseFloat(data.toString()) - north) % 360;
			    resolve(result);
		        }else {
			    resolve("please calibrate first");
		        }
	            });
		//});
	    });
	}

	private get_light_level(){
	    return new Promise((resolve, reject) => {
		const process = spawn('python', ['./lib/enviro_phat.py', 'read_light_level']);
		process.stdout.on('data', (data) => {
		    resolve(parseFloat(data.toString()));
	        }); 
	    });
	}

	private get_rgb_values(){
	    return new Promise((resolve, reject) => {
		const process = spawn('python', ['./lib/enviro_phat.py', 'read_rgb_values']);
		process.stdout.on('data', (data) => {
		    resolve(data.toString());
	        }); 
	    });	
	}

	private add_properties(){
		//Property: temperature
		this.thing.addProperty(
			"temperature",
			{
				type: "number",
				unit: "Celsius",
				readOnly: true,
				description: "Read the current temperature value"			
			}
		);		
		this.thing.setPropertyReadHandler("temperature",this.get_temp)

		//Property: pressure
		this.thing.addProperty(
			"pressure",
			{
				type: "number",
				unit: "hPa",
				readOnly: true,
				description: "Read the current pressure value"			
			}
		);		
		this.thing.setPropertyReadHandler("pressure",this.get_pressure)		

		//Property: motion
		this.thing.addProperty(
			"motion",
			{
				type: "string",
				readOnly: true,
				description: "Read the current position values"
			}
		);
		this.thing.setPropertyReadHandler("motion",this.get_motion)

		//Property: heading
		this.thing.addProperty(
			"heading",
			{
				type: "number",
				readOnly: true,
				description: "Read the current heading position, needs calibration action first to return correct values"			
			}
		);		
		this.thing.setPropertyReadHandler("heading",this.get_heading)

		//Property: north
		this.thing.addProperty(
			"north",
			{
				type: "number",
				readOnly: false,
				writeOnly: false,
				description: "Use for user to calibrate direction"
			}
		,-1);
		
		//Property: light level
		this.thing.addProperty(
			"lightLevel",
			{
				type: "number",
				unit: "lux",
				readOnly: true,
				description: "Read the current heading position"			
			}
		);		
		this.thing.setPropertyReadHandler("lightLevel",this.get_light_level)

		//Property: rgb values
		this.thing.addProperty(
			"rgbValues",
			{
				type: "string",
				readOnly: true,
				description: "Read the current heading position"			
			}
		);		
		this.thing.setPropertyReadHandler("rgbValues",this.get_rgb_values)		
	}

//----------------------------------- Actions -----------------------------------------
	private led_on(){
	    return new Promise((resolve, reject) => {
		const process = spawn('python', ['./lib/enviro_phat.py', 'led_on']);
			process.stdout.on('data', (data) => {
			resolve(data.toString());
        	}); 
            });
	}

	private led_off(){
	    return new Promise((resolve, reject) => {
		const process = spawn('python', ['./lib/enviro_phat.py', 'led_off']);
			process.stdout.on('data', (data) => {
			resolve(data.toString());
        	}); 
            });
	}

	private calibrate(){
	    return new Promise((resolve, reject) => {
		const process = spawn('python', ['./lib/enviro_phat.py', 'read_heading']);
		process.stdout.on('data', (data) => {
		    //that.thing.properties["north"].write(parseFloat(data.toString()));
		    north = parseFloat(data.toString());
		    resolve("calibrated");
		});
	    });
	}
	
	private add_actions(){
	    this.thing.addAction(
		"ledOn",
		{output:{const:"turned leds on\n"} },
		() => { return this.led_on();}	
	    );
	    this.thing.addAction(
		"ledOff",
		{output:{const:"turned leds off\n"}  },
		() => { return this.led_off();}	
	    );
	    this.thing.addAction(
		"calibrate",
		{output:{const:"calibrated"}  },
		() => { return this.calibrate();}
	    );
	}

}


