import * as WoT from "wot-typescript-definitions"

var request = require('request');

const Ajv = require('ajv');
var ajv = new Ajv();

const A4988 = require('/home/pi/Desktop/Stepper_Motor/A4988');
const a4988 = new A4988({ step: 15, dir: 14, ms1: 24, ms2: 23, ms3: 18, enable: 25 }); 

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
                id : "urn:dev:ops:32473-StepperMotor-001",
                title : "StepperMotor",
                description : "Stepper motor on a Rpi",
                securityDefinitions: { 
                    "nosec_sc": { 
                        "scheme": "nosec" 
                    }
                },
                security: "nosec_sc",
				properties:  {
					speed: {
						"title": "Speed",
						"description": "Speed of the conveyor belt",
						"type": "integer",
						"unit": "%",
						"minimum": 1,
						"maximum": 100,
						"readOnly": false,
						"writeOnly": false,
						"observable": false
					}
				},
				actions:{
					startBeltForward: { 
						"title": "Start conveyor belt",
						"description": "This action starts moving the conveyor belt forward", 
						"output": {
							"const": "Conveyor belt started"
						},
						"idempotent": false,
						"safe": false
					},
					startBeltBackward: { 
						"title": "Start conveyor belt",
						"description": "This action starts moving the conveyor belt backward", 
						"output": {
							"const": "Conveyor belt started"
						},
						"idempotent": false,
						"safe": false
					},
					stopBelt: { 
						"title": "Stop conveyor belt",
						"description": "This action stops moving the conveyor belt", 
						"output": {
							"const": "Conveyor belt stopped"
						},
						"idempotent": false,
						"safe": false
					}     
				}
            }
        ).then((exposedThing)=>{
			this.thing = exposedThing;
			console.log("thing is ",this.thing)
			this.td = exposedThing.getThingDescription();
		    this.add_properties();
			this.add_actions();
			this.thing.expose();
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

	// ------------------------  Properties  ------------------------

	// Writing and reading the property value of "speed" is done by the default read- and writehandler; 

    private setSpeedStepperMotor(newSpeed){
		return new Promise((resolve, reject) => {
			// read something
			a4988.turn_speed((101 - newSpeed) / 100);
			resolve();
		});
    }

	// ------------------------    Actions   ------------------------

	private startBeltForward(){
		return new Promise((resolve, reject) => {
				a4988.turn_direction(true);
				a4988.turn_step_size('FULL');
				a4988.turn(1); 
				resolve();
			});				
	}
    private startBeltBackward(){
		return new Promise((resolve, reject) => {
				a4988.turn_direction(false);
				a4988.turn_step_size('FULL');
				a4988.turn(1);			
				resolve();
			});
	}
	private stopBelt(){
		return new Promise((resolve, reject) => {
				a4988.stop();
				resolve();
			});	
    }

	// --------------------------  Events  --------------------------

	// -----------------------  ADD P, A & E  -----------------------
	 
    private add_properties() {
        //fill in add properties
		this.thing.writeProperty("speed", 1); 	// replace quotes with the initial value
		a4988.turn_speed((101 - 1) / 100);		// default value is 1 (min value)
        this.thing.setPropertyWriteHandler("speed", (newSpeed) => {       
			return new Promise((resolve, reject) => {
			   if (!ajv.validate(this.td.properties.speed, newSpeed)) {	
				   reject(new Error ("Invalid input"));
			   }
			   else {
				//    resolve(this.speedWriteHandler(newSpeed));
				this.setSpeedStepperMotor(newSpeed);
				resolve(newSpeed);
			   }
		   });
	   });
    }

    private add_actions() {
        //fill in add actions
        this.thing.setActionHandler("startBeltForward", () => {            
			return this.startBeltForward();
		});
		this.thing.setActionHandler("startBeltBackward", () => {            
			return this.startBeltBackward();
		});
		this.thing.setActionHandler("stopBelt", () => {            
			return this.stopBelt();
	    });
	}
}
