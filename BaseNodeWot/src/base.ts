import * as WoT from "wot-typescript-definitions"

var request = require('request');

const Ajv = require('ajv');
var ajv = new Ajv();

export class WotDevice {
    public thing: WoT.ExposedThing;
    public factory: WoT.WoTFactory;
    constructor(thingFactory: WoT.WoTFactory, tdDirectory?: string) {
        //create WotDevice as a server
        this.factory = thingFactory;
        this.thing = thingFactory.produce(
            //fill in the empty quotation marks
            `{
                "@context": [
                    "https://www.w3.org/2019/wot/td/v1",
                    { "@language" : "en" }],
                "@type": "",
                "title" : "",
                "description" : "",
                "securityDefinitions": { 
                    "": { 
                        "scheme": "" 
                    }
                },
                "security": ""
            }`
            );
        this.thing.id = "new:thing"; //change the id to something unique in your case
        this.add_properties();
        this.add_actions();
        this.add_events();
		this.listen_to_myEvent(); //used to listen to specific events provided by a library. If you don't have events, simply remove it
        this.thing.expose();

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

    private myPropertyHandler(){
		return new Promise((resolve, reject) => {
			// read something
			resolve();
		});
    }

    private myActionHandler(){
		return new Promise((resolve, reject) => {
			// do something
			resolve();
		});	
    }

    private listen_to_myEvent() {
		specialLibrary.getMyEvent()
		.then((thisEvent) => {
			this.thing.events[""].emit(direction);
    	});
	}

    private add_properties() {
        //fill in add properties
        /*
		this.thing.addProperty(
			"",
			{
				title:"",
				description: "",
				type: ""
			},
			0
		);
		this.thing.setPropertyReadHandler("", this.myPropertyHandler)
		*/
    }

    private add_actions() {
        //fill in add actions
		/*
        this.thing.addAction(
            "myAction", 
            {
		        title:"",
		        description: ""
            }, 
            (inputData) => { 
		         return new Promise((resolve, reject) => {
		            if (!ajv.validate(this.thing.actions.myAction.input, inputData)) {
		                reject(new Error ("Invalid input"));
		            }
		            else {
		                resolve(this.myActionHandler(inputData));
		            }
		        });
            }
        );
		*/
    }
    private add_events() {
        //fill in add events
		/*
		this.thing.addEvent(
		        "",
		        {
		            "data":{				
						"type": ""
					}	            
				}
		    );
		*/
    }
}