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
            {
                "@context": [
                    "https://www.w3.org/2019/wot/td/v1",
                    { "@language" : "en" }],
                "@type": "",
                id : "urn:dev:ops:32473-DobotMagician-001",
                title : "DobotMagician",
                description : "Robot arm on a sliding rail",
                properties:{
					position:{
						type:"object",
						description:"Position of the end effector. x,y,z are dependent of each other",
						properties:{
							x:{
								type:"number",
								maximum:180
								minimum: 0
							},
							y:{
								type:"number",
								maximum:180
								minimum: 0
							},
							z:{
								type:"number",
								maximum:45
								minimum: -110
							},
							r:{
								type:"number",
								maximum:180
								minimum: -180
							},
							l:{
								type:"number",
								maximum:1000
								minimum: 0
							}
						}
					}
				},
				actions:{
                    moveToStartPosition: {
						title: "Move to Start Position",
                        description: "The robot moves to the default start position", 
                        output: {"const": "At the start position"},
                        synchronous:true,
                        idempotent: false,
                        safe: false,
                    },
					pickObjectPosition1: { 
						title: "Pick Object at Position 1",
						description: "The robot picks the object at the preprogrammed position 1", 
						output: {"const": "Dobot magician is picking object at position 1"},
						synchronous:true,
						idempotent: false,
						safe: false,
					},
					pickObjectPosition2: { 
						title: "Pick Object at Position 2",
						description: "The robot picks the object at position 2", 
						output: {"const": "Dobot magician is picking object at position 2"},
						synchronous:true,
						idempotent: false,
						safe: false,
                    },
                    moveToColorSensor1: {
						title: "Move to Color Sensor 1",
                        description: "Robot moves the object to the preprogrammed location of color sensor 1",
                        output: {"const": "Dobot magician is moving object to the color sensor 1"},
                        synchronous:true,
                        idempotent: false,
                        safe: false, 
                    },
                    moveToColorSensor2: {
						title: "Move to Color Sensor 2",
                        description: "Robot moves the object to the preprogrammed location of color sensor 2",
                        output: {"const": "Dobot magician is moving object to the color sensor 2"},
                        synchronous:true,
                        idempotent: false,
                        safe: false,
                    },
                    moveToObjectRed: {
						title:"Move to Object Red",
                        description: "Robot moves the object to the preprogrammed location for red colored objects",
                        output: {"const": "Dobot magician is moving object at the color dependent position"},
                        synchronous:true,
                        idempotent: false,
                        safe: false,
                    },
                    moveToObjectGreen: { 
                        title: "Move to Object Green",
                        description: "Robot moves the object to the preprogrammed location for green colored objects",
                        output: {"const": "Dobot magician is moving object at the color dependent position"},
                        synchronous:true,
                        idempotent: false,
                        safe: false,
                    },
                    moveToObjectBlue: { 
                        title: "Move to Object Blue",
                        description: "Robot moves the object to the preprogrammed location for blue colored objects",
                        output: {"const": "Dobot magician is moving object at the color dependent position"},
                        synchronous:true,
                        idempotent: false,
                        safe: false,
                    },
                    moveToObjectNone: { 
                        title: "Move to Object None",
                        description: "The robot moves the object to preprogrammed position for objects whose color cannot be detected",
                        output: {"const": "Dobot magician is moving object at the color dependent position"},
                        synchronous:true,
                        idempotent: false,
                        safe: false,
                    }
				}
            }
        ).then((exposedThing)=>{
			this.thing = exposedThing;
			this.td = exposedThing.getThingDescription();
			this.addPropertyHandlers();
			this.addActionHandlers();
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
    private getPosition(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/getPosition.py']);
				process.stdout.on('data', (data) => {
				resolve(JSON.parse(data));
			});			
		});	
	}
	// ------------------------    Actions   ------------------------

    private actionHandlerStartPosition(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/startPosition.py']);
				process.stdout.on('data', (data) => {
				resolve("At the start position");
			});			
		});	
	}
    private actionHandlerPickObjectPosition1(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/pickObjectPosition1.py']);
				process.stdout.on('data', (data) => {
				resolve("Picked object at position 1");
			});			
		});	
	}
    private actionHandlerPickObjectPosition2(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/pickObjectPosition2.py']);
				process.stdout.on('data', (data) => {
				resolve("Picked object at position 2");
			});
		});	
    }
    private actionHandlerMoveToColorSensor1(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/moveToColorSensor1.py']);
				process.stdout.on('data', (data) => {
				resolve("At color sensor 1");
			});
		});	
    }
    private actionHandlerMoveToColorSensor2(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/moveToColorSensor2.py']);
				process.stdout.on('data', (data) => {
				resolve("At color sensor 2");
			});
		});	
    }
    private actionHandlerMoveObjectRed(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/moveObjectRed.py']);
				process.stdout.on('data', (data) => {
				resolve("At red object location");
			});
		});	
    }
    private actionHandlerMoveObjectGreen(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/moveObjectGreen.py']);
				process.stdout.on('data', (data) => {
				resolve("At green object location");
			});
		});	
    }
    private actionHandlerMoveObjectBlue(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/moveObjectBlue.py']);
				process.stdout.on('data', (data) => {
				resolve("At blue object location");
			});
		});	
    }
    private actionHandlerMoveObjectNone(){
		return new Promise((resolve, reject) => {
			const process = spawn('python3', ['./lib/moveObjectNone.py']);
				process.stdout.on('data', (data) => {
				resolve("At undefined color object location");
			});
		});	
    }

	// --------------------------  Events  --------------------------

 	// -----------------------  ADD P, A & E  -----------------------

	private addPropertyHandlers(){
		this.thing.setPropertyReadHandler("position", this.getPosition);
	}

    private addActionHandlers() {
        //fill in add actions
        this.thing.setActionHandler("moveToStartPosition", () => {            
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
        this.thing.setActionHandler("moveToObjectRed", () => {            
			return this.actionHandlerMoveObjectRed();
        });
        this.thing.setActionHandler("moveToObjectGreen", () => {            
			return this.actionHandlerMoveObjectGreen();
        });
        this.thing.setActionHandler("moveToObjectBlue", () => {            
			return this.actionHandlerMoveObjectBlue();
        });
        this.thing.setActionHandler("moveToObjectNone", () => {            
			return this.actionHandlerMoveObjectNone();
        });
    }
}

