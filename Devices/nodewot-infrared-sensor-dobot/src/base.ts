import * as WoT from "wot-typescript-definitions"

var request = require('request');

const Gpio = require('onoff').Gpio;
const infraredSensor = new Gpio(17, 'in', 'both', {debounceTimeout: 10});
const risingEdge_infraredSensor = new Gpio(17, 'in', 'rising', {debounceTimeout: 10});

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
                id : "urn:dev:ops:32473-InfraredSensor-001",
                title : "InfraredSensor",
                description : "Infrared sensor for the detection of objects",
                securityDefinitions: { 
                    "nosec_sc": { 
                        "scheme": "nosec_sc" 
                    }
                },
                security: "nosec_sc",
    properties:{
                objectPresence:{
				title:"Read infrared sensor",
				description: "Reads the infrared sensor; 0: no object in front; 1: object in front.",
                            type: "boolean",
                            observable: true
					}
				},
	            
	events:{
                detectedObject:{
				title:"Object detected",
				description: "Detects the rising edge of the signal of the infrared sensor.",
				data:{
					type: "boolean"
				}
							
			}
		}
	
            }
        ).then((exposedThing)=>{
		    this.thing = exposedThing;
		    this.td = exposedThing.getThingDescription();
    		this.add_properties();
            this.thing.expose();

		    if (tdDirectory) { this.register(tdDirectory); }
        });

		this.listen_to_detectedObjectEvent(); //used to listen to specific events provided by a library. If you don't have events, simply remove it
        this.listen_to_objectPresenceProperty();

        if (tdDirectory) { this.register(tdDirectory); }

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

    private get_objectPresence_Handler(){
		return new Promise((resolve, reject) => {
            // GET -> 0: no object in front; 1: object in front
            var value = infraredSensor.readSync(); 
			resolve(1-value);
		});
    }
    
    private listen_to_objectPresenceProperty() {
        // OBSERVEABLE -> 0: no object in front; 1: object in front
        infraredSensor.watch((err, value) => {
			this.thing.writeProperty("objectPresence", 1-value);
			console.log(1-value);
		});
	}

    // --------------------------  Events  --------------------------

    private listen_to_detectedObjectEvent() {
        // SUBCRIBEEVENT -> Detects the rising edge of the signal of the infrared sensor.
        risingEdge_infraredSensor.watch((err, value) => {
			this.thing.emitEvent("detectedObject", 1-value);
			console.log(1-value);
		});
	}

    // -----------------------  ADD P, A & E  -----------------------

    private add_properties() {
        //fill in add properties
        this.thing.writeProperty("objectPresence", this.get_objectPresence_Handler); //replace quotes with the initial value
	    this.thing.setPropertyReadHandler("objectPresence", this.get_objectPresence_Handler);
    }

}