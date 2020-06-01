import * as WoT from "wot-typescript-definitions"

var request = require('request');
const Ajv = require('ajv');
var PanTilt = require('pan-tilt-hat');

var pantilt = new PanTilt();
pantilt.goto_home();

var ajv = new Ajv();

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
              title : "PanTilt",
              id : "esi:pantilt:1",
              description : "A Pan and Tilt platform on top of a Raspberry",
              properties:{
                	panPosition:{
							title:"Pan Position",
							readOnly: true,
							description: "The current position of the pan platform in degrees",
							unit: "degrees",
							type: "number",
							minimum: -90.0,
							maximum: 90.0
					},
					tiltPosition:{
						title:"Tilt Position",
						readOnly: true,
						description: "The current position of the tilt platform in degrees",
						unit: "degrees",
						type: "number",
						minimum: -80.0,
						maximum: 80.0
					},
                },
                actions:{
		            goHome:{
						title:"Go Home",
						description: "Returns the pan and tilt to their home position which is at 0 and 0 degrees"
				    },
				    scan:{
						title:"Scan",
						description: "Scans left and right once, starting from the leftmost point. Tilt stays the same"
				    },
				    panTo:{
						description: "Moves the pan platform to the angle specific in the input",
						title:"Pan To",
						input:{
							unit: "degrees",
							type: "number",
							minimum: -90.0,
							maximum: 90.0
		        		}
            		},
            		tiltTo:{
						description: "Moves the tilt platform to the angle specific in the input",
						title:"Tilt To",
						input:{
							unit: "degrees",
							type: "number",
							minimum: -80.0,
							maximum: 80.0
						}
					},
					moveTo:{
						description: "Moves the tilt and pan platform to the angles given in input",
						title:"Move To",
						input:{
							type:"object",
							properties:{
								tiltAngle:{
									title:"Tilt To",
									unit: "degrees",
									type: "number",
									minimum: -80.0,
									maximum: 80.0
								},
								panAngle:{
									title:"Pan To",
									unit: "degrees",
									type: "number",
									minimum: -90.0,
									maximum: 90.0
								}
							},
							required:["panAngle","tiltAngle"]
		        		}
            		},
            		panContinuously:{
						description: "Moves the pan platform with speed given in input until a stop action is invoked or limits are reached",
						title:"Pan Continuously",
						input:{
							description:"The speed at which the platform moves. Negative values for right and positive values for left",
							unit: "angle per sec",
							type: "number",
							minimum: -15.0,
							maximum: 15.0
						}
					},
					tiltContinuously:{
						description: "Moves the tilt platform with speed given in input until a stop action is invoked or limits are reached",
						title:"Tilt Continuously",
						input:{
							description:"The speed at which the platform moves. Negative values for moving up and positive values for moving down",
							unit: "angle per sec",
							type: "number",
							minimum: -15.0,
							maximum: 15.0
						}
				    },
				    moveContinuously:{
						description: "Moves the tilt and pan platform with the speeds given in input until a stop action is invoked or limits are reached",
						title:"Move Continuously",
						input:{
							type:"object",
							properties:{
								tiltSpeed:{
									description:"The speed at which the tilt platform moves. Negative values for moving up and positive values for moving down",
									unit: "angle per sec",
									type: "number",
									minimum: -15.0,
									maximum: 15.0
								},
								panSpeed:{
									description:"The speed at which the platform moves. Negative values for right and positive values for left",
									unit: "angle per sec",
									type: "number",
									minimum: -15.0,
									maximum: 15.0
								}
							},
							required:["panSpeed","tiltSpeed"]
						}
				    },
	                stopMovement:{
						title:"Stop Movement",
						description: "Stops any movement that was created with continuous movement calls"
				    }
                }
            }
        ).then((exposedThing)=>{
		    	this.thing = exposedThing;
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
    
    private getPanAngle() {
        return new Promise((resolve, reject) => {
            resolve(pantilt.pan_position);
         });
    }
    
    private getTiltAngle() {
        return new Promise((resolve, reject) => {
            resolve(pantilt.tilt_position);
         });
    }

    private gohome() {
		return new Promise((resolve, reject) => {
			pantilt.goto_home();
			resolve();
		});
	}
	
	private panTo(angle) {
		return new Promise((resolve, reject) => {
			pantilt.pan(angle);
			resolve();
		});
	}
	
	private tiltTo(angle) {
		return new Promise((resolve, reject) => {
			pantilt.tilt(angle);
			resolve();
		});
	}
	
	private pan(speed) {
		return new Promise((resolve, reject) => {
			if(speed > 0 ) {
				pantilt.pan_left(speed);
				resolve();
			} else {
				pantilt.pan_right(-speed);
				resolve();
			}
		});
	}
	
	private tilt(speed) {
		return new Promise((resolve, reject) => {
			if(speed > 0 ) {
				pantilt.tilt_down(speed);
				resolve();
			} else {
				pantilt.tilt_up(-speed);
				resolve();
			}
		});
	}
	
	private moveTo(angles) {
		return new Promise((resolve, reject) => {
			this.tiltTo(angles.tiltAngle).then(()=>{
				this.panTo(angles.panAngle).then(()=>{
					resolve()
				}).catch(() => reject());
			}).catch(() => reject());
			
		});
	}
	
	private move(speeds) {
		return new Promise((resolve, reject) => {
			this.tilt(speeds.tiltSpeed).then(()=>{
				this.pan(speeds.panSpeed).then(()=>{
					resolve()
				}).catch(() => reject());
			}).catch(() => reject());
			
		});
	}

	private scan() {
    	var anotherThis = this;
        return new Promise((resolve, reject) => {
            anotherThis.panTo(90).then(() => {
                anotherThis.pan(-10).then(() => {
                    setTimeout(function () {
                        anotherThis.pan(10).then(() => {
                            setTimeout(function () {
                                anotherThis.stopMovement().then(() => {
                                    resolve();
                                }).catch(() => reject());
                            }, 20000);
                        }).catch(() => reject());
                    }, 20000);
                }).catch(() => reject());
            }).catch(() => reject());
        });
    }
	
	private stopMovement() {
		return new Promise((resolve, reject) => {
			pantilt.stop();
			resolve();
		});
	}

    private add_properties() {

		this.thing.writeProperty("panPosition",pantilt.pan_position);
		this.thing.setPropertyReadHandler("panPosition", this.getPanAngle);
		
		this.thing.writeProperty("tiltPosition",pantilt.tilt_position);
		this.thing.setPropertyReadHandler("tiltPosition", this.getTiltAngle)
    }
    
    private add_actions() {

		this.thing.setActionHandler("goHome", (params) => { 
           	return this.gohome(); 
        });
        
        this.thing.setActionHandler("scan",(params) => { 
        	return this.scan(); 
    	});
        
        this.thing.setActionHandler("panTo",(angle) => {            
         	return new Promise((resolve, reject) => {
	            if (!ajv.validate(this.td.actions.panTo.input, angle)) {
	                reject(new Error ("Invalid input"));
	            }
	            else {
	                resolve(this.panTo(angle));
	            }
	        });
        });
        
        
        this.thing.setActionHandler("tiltTo", (angle) => { 		         
        	return new Promise((resolve, reject) => {
	            if (!ajv.validate(this.td.actions.tiltTo.input, angle)) {
	                reject(new Error ("Invalid input"));
	            }
	            else {
	                resolve(this.tiltTo(angle));
	            }
	        }); 
        });
        
        this.thing.setActionHandler("moveTo",(angles) => {
	         return new Promise((resolve, reject) => {
	            if (!ajv.validate(this.td.actions.moveTo.input, angles)) {
	                reject(new Error ("Invalid input"));
	            }
	            else {
	                resolve(this.moveTo(angles));
	            }
	        });
        });
        
    	this.thing.setActionHandler("panContinuously",(speed) => {
	         return new Promise((resolve, reject) => {
	         	if (speed == 0) resolve();
	            if (!ajv.validate(this.td.actions.panContinuously.input, speed)) {
	                reject(new Error ("Invalid input"));
	            }
	            else {
	                resolve(this.pan(speed));
	            }
	        });
        });
        
    	this.thing.setActionHandler("tiltContinuously",(speed) => {
        	return new Promise((resolve, reject) => {
		         	if (speed == 0) resolve();
		            if (!ajv.validate(this.td.actions.tiltContinuously.input, speed)) {
		                reject(new Error ("Invalid input"));
		            }
		            else {
		                resolve(this.tilt(speed));
		            }
		        });
        });
        
    	this.thing.setActionHandler("moveContinuously",(speeds) => {
	         return new Promise((resolve, reject) => {
	         	if ((speeds.panSpeed == 0)&&(speeds.tiltSpeed == 0)) resolve();
	            if (!ajv.validate(this.td.actions.moveContinuously.input, speeds)) {
	                reject(new Error ("Invalid input"));
	            }
	            else {
	                resolve(this.move(speeds));
	            }
	        });
        });
        
        this.thing.setActionHandler("stopMovement",() => { 
        	return this.stopMovement(); 
        });
        
    }
}
