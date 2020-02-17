import * as WoT from "wot-typescript-definitions"

var request = require('request');

const { spawn } = require('child_process')

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
                id : "urn:dev:ops:32473-DobotMagician-001",
                title : "DobotMagician",
                description : "Dobot Magician control for ",
                securityDefinitions: { 
                    "": { 
                        "scheme": "" 
                    }
                },
                security: "",
				actions:{
                    startPosition: { 
                        title: "Default start position",
                        description: "The robot moves to the default start position", 
                        output: {
                            "const": "Dobot magician is moving to the default start position"
                        },
                        synchronous:true,
                        idempotent: false,
                        safe: false,
                    },
					pickObjectPosition1: { 
						title: "Pick object position 1",
						description: "The robot picks the object at position 1", 
						output: {
							"const": "Dobot magician is picking object at position 1"
                        },
                        synchronous:true,
						idempotent: false,
						safe: false,
					},
					pickObjectPosition2: { 
						title: "Pick object position 2",
						description: "The robot picks the object at position 2", 
						output: {
							"const": "Dobot magician is picking object at position 2"
                        },
                        synchronous:true,
						idempotent: false,
						safe: false,
                    },
                    moveToColorSensor1: { 
                        title: "Moves object to color sensor 1",
                        description: "The robot moves the object at the color sensor 1 for reading the color of the object", 
                        output: {
                            "const": "Dobot magician is moving object at the color sensor 1"
                        },
                        synchronous:true,
                        idempotent: false,
                        safe: false, 
                    },
                    moveToColorSensor2: { 
                        title: "Moves object to color sensor 2",
                        description: "The robot moves the object at the color sensor 2 for reading the color of the object", 
                        output: {
                            "const": "Dobot magician is moving object at the color sensor 2"
                        },
                        synchronous:true,
                        idempotent: false,
                        safe: false,
                    },
                    moveObjectRed: { 
                        title: "Moves object to position red-color",
                        description: "The robot moves the object to a colot dependend position", 
                        output: {
                            "const": "Dobot magician is moving object at the color dependent position"
                        },
                        synchronous:true,
                        idempotent: false,
                        safe: false,
                    },
                    moveObjectGreen: { 
                        title: "Moves object to position green-color",
                        description: "The robot moves the object to a colot dependend position", 
                        output: {
                            "const": "Dobot magician is moving object at the color dependent position"
                        },
                        synchronous:true,
                        idempotent: false,
                        safe: false,
                    },
                    moveObjectBlue: { 
                        title: "Moves object to position blue-color",
                        description: "The robot moves the object to a colot dependend position", 
                        output: {
                            "const": "Dobot magician is moving object at the color dependent position"
                        },
                        synchronous:true,
                        idempotent: false,
                        safe: false,
                    },
                    moveObjectNone: { 
                        title: "Moves object to position none-color",
                        description: "The robot moves the object to a colot dependend position", 
                        output: {
                            "const": "Dobot magician is moving object at the color dependent position"
                        },
                        synchronous:true,
                        idempotent: false,
                        safe: false,
                    }
				}
            }
        ).then((exposedThing)=>{
			this.thing = exposedThing;
			this.td = exposedThing.getThingDescription();
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

	// ------------------------    Actions   ------------------------

    private actionHandlerStartPosition(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/startPosition.py']);
				process.stdout.on('data', (data) => {
				resolve();
			});			
		});	
	}
    private actionHandlerPickObjectPosition1(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/pickObjectPosition1.py']);
				process.stdout.on('data', (data) => {
				resolve();
			});			
		});	
	}
    private actionHandlerPickObjectPosition2(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/pickObjectPosition2.py']);
				process.stdout.on('data', (data) => {
				resolve();
			});
		});	
    }
    private actionHandlerMoveToColorSensor1(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/moveToColorSensor1.py']);
				process.stdout.on('data', (data) => {
				resolve();
			});
		});	
    }
    private actionHandlerMoveToColorSensor2(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/moveToColorSensor2.py']);
				process.stdout.on('data', (data) => {
				resolve();
			});
		});	
    }
    private actionHandlerMoveObjectRed(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/moveObjectRed.py']);
				process.stdout.on('data', (data) => {
				resolve();
			});
		});	
    }
    private actionHandlerMoveObjectGreen(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/moveObjectGreen.py']);
				process.stdout.on('data', (data) => {
				resolve();
			});
		});	
    }
    private actionHandlerMoveObjectBlue(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/moveObjectBlue.py']);
				process.stdout.on('data', (data) => {
				resolve();
			});
		});	
    }
    private actionHandlerMoveObjectNone(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/moveObjectNone.py']);
				process.stdout.on('data', (data) => {
				resolve();
			});
		});	
    }

	// --------------------------  Events  --------------------------

 	// -----------------------  ADD P, A & E  -----------------------

    private add_actions() {
        //fill in add actions
        this.thing.setActionHandler("startPosition", () => {            
			return this.actionHandlerStartPosition();
		});
        this.thing.setActionHandler("pickObjectPosition1", () => {            
			return this.actionHandlerPickObjectPosition1();
		});
		this.thing.setActionHandler("pickObjectPosition2", () => {            
			return this.actionHandlerPickObjectPosition2();
        });
        this.thing.setActionHandler("moveToColorSensor1", () => {            
			return this.actionHandlerMoveToColorSensor1();
        });
        this.thing.setActionHandler("moveToColorSensor2", () => {            
			return this.actionHandlerMoveToColorSensor2();
        });
        this.thing.setActionHandler("moveObjectRed", () => {            
			return this.actionHandlerMoveObjectRed();
        });
        this.thing.setActionHandler("moveObjectGreen", () => {            
			return this.actionHandlerMoveObjectGreen();
        });
        this.thing.setActionHandler("moveObjectBlue", () => {            
			return this.actionHandlerMoveObjectBlue();
        });
        this.thing.setActionHandler("moveObjectNone", () => {            
			return this.actionHandlerMoveObjectNone();
        });
    }
}

