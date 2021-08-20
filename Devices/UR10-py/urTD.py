## user manuel pages, 81, 93, X=1300 , Z= 1000, Y=-1300
def get_td(HTTP_ip_address):
	return{
		"@context": "https://www.w3.org/2019/wot/td/v1",
	    "id": "de:tum:ei:esi:ur10", 
	    "title": "Ur-10",
	    "securityDefinitions": { "nosec_sc": {"scheme": "nosec"}
	    },
	    "security": ["nosec_sc"],
	    "properties": {
	    	"homeLoc":{
	    		"title":"Home location",
	    		"description":"Home location on x-y-z space and joint degrees values",
	    		"type":"object",
	    		"required":["x","y","z"],
	    		"properties":{
	    			"x":{
						"type":"integer"
						},
	    			"y":{
						"type":"integer"
						},
	    			"z":{
						"type":"integer"
						},
					"base":{
						"type":"number"
					},
	    			"shoulder":{
						"type":"number"
					},
	    			"elbow":{
						"type":"number"
					},
					"wrist1":{
						"type":"number"
					},
					"wrist2":{
						"type":"number"
					},
					"wrist3":{
						"type":"number"
					}
	    		},
				"forms": [
					{
					"href": "http://{}/ur10/properties/homeloc".format(HTTP_ip_address),
					"op": ["readproperty"],
					"contentType":"application/json"
					}
				],
				"readOnly": True
	    	},
			"curLocation":{
	    		"title":"Location",
	    		"description":"Gives back the current location in cartesian coordinates",
	    		"type":"object",
	    		"required":["x","y","z"],
	    		"properties":{
	    			"x":{
						"type":"integer",
						"minimum": -1300,
						"maximum": 1300
						},
	    			"y":{
						"type":"integer",
						"minimum": -1300,
						"maximum": 1300
						},
	    			"z":{
						"type":"integer",
						"minimum": -1300,
						"maximum": 1300
						}
	    		},
				"forms": [
					{"href": "http://{}/ur10/properties/curLocation".format(HTTP_ip_address),
					"op": ["readproperty"],
					"contentType":"application/json"
					}
				],
				"readOnly": True
	    	},
			"curJointPos":{
	    		"title":"Current Joint Position",
	    		"description":"Gives back the current degree values for joints",
	    		"type":"object",
	    		"required":["base","shoulder","elbow","wrist1","wrist2","wrist3"],
	    		"properties":{
	    			"base":{
						"type":"number",
						"minimum": -360,
						"maximum": 360
					},
	    			"shoulder":{
						"type":"number",
						"minimum": -360,
						"maximum": 360
					},
	    			"elbow":{
						"type":"number",
						"minimum": -360,
						"maximum": 360
					},
					"wrist1":{
						"type":"number",
						"minimum": -360,
						"maximum": 360
					},
					"wrist2":{
						"type":"number",
						"minimum": -360,
						"maximum": 360
					},
					"wrist3":{
						"type":"number",
						"minimum": -360,
						"maximum": 360
					}
	    		},
				"forms": [
					{"href": "http://{}/ur10/properties/curJointPos".format(HTTP_ip_address),
					"op": ["readproperty"],
					"contentType":"application/json"
					}
				],
				"readOnly": True
	    	},
            "moveSpeed":{
                "title":"Speed",
                "description": "Its value sets the moving speed of all UR-10 robot arm joints, default value is 0.5 but can be changed",
                "type": "number",
                "minimum": 0.1, 
                "maximum": 1.0,
                "forms": [
					{"href": "http://{}/ur10/properties/moveSpeed".format(HTTP_ip_address),
					"op": "readproperty",
					"contentType":"application/json",
					"htv:methodName": "GET",
					"security": "nosec_sc"
					},
					{"href": "http://{}/ur10/properties/moveSpeed".format(HTTP_ip_address),
					"htv:methodName": "PUT",
                    "op": "writeproperty",
					"contentType":"application/json",
					"security": "nosec_sc"
					}
				],
				"readOnly": False,
				"writeOnly": False
            },
			"moveAcceleration":{
                "title":"Acceleration",
                "description": "Its value sets the moving acceleration of all UR-10 robot arm joints, default value is 0.5 but can be changed",
                "type": "number",
                "minimum": 0.1, 
                "maximum": 1.0,
                "forms": [
					{"href": "http://{}/ur10/properties/moveAcceleration".format(HTTP_ip_address),
					"op": "readproperty",
					"contentType":"application/json",
					"htv:methodName": "GET",
					"security": "nosec_sc"
					},
					{"href": "http://{}/ur10/properties/moveAcceleration".format(HTTP_ip_address),
					"htv:methodName": "PUT",
                    "op": "writeproperty",
					"contentType":"application/json",
					"security": "nosec_sc"
					}
				],
				"readOnly": False,
				"writeOnly": False
            }
	    },
	
	    "actions":{
			"goHome":{
				"title":"Go Home",
				"description":"Go to the home position",
				"forms":[{
					"href":"http://{}/ur10/actions/gohome".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": False,
				"safe": False
			},	
			"turnBase":{
				"title":"Turn Base",
				"description":"Turn the robot arm Base joint a certain amount within borders",
				"input":{
					"type":"object",
					"required":["base"],
					"properties":{
						
						"base":{
							"type":"number",
							"minimum": -360,
							"maximum": 360
						}
					}
	    		},

				"forms":[{
					"href":"http://{}/ur10/actions/turnBase".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": False,
				"safe": False
			},
            "turnShoulder":{
				"title":"Turn Shoulder",
				"description":"Turn the robot arm shoulder joint a certain amount within borders",
				"input":{
					"type":"object",
					"required":["shoulder"],
					"properties":{
						
						"shoulder":{
							"type":"number",
							"minimum": -360,
							"maximum": 360
						}
					}
	    		},

				"forms":[{
					"href":"http://{}/ur10/actions/turnShoulder".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": False,
				"safe": False
			},
            "turnElbow":{
				"title":"Turn Elbow",
				"description":"Turn the robot arm elbow joint a certain amount within borders",
				"input":{
					"type":"object",
					"required":["elbow"],
					"properties":{
						
						"elbow":{
							"type":"number",
							"minimum": -360,
							"maximum": 360
						}
					}
	    		},

				"forms":[{
					"href":"http://{}/ur10/actions/turnElbow".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": False,
				"safe": False
			},
            "turnWrist1":{
				"title":"Turn Wrist1",
				"description":"Turn the robot arm Wrist1 joint a certain amount within borders",
				"input":{
					"type":"object",
					"required":["wrist1"],
					"properties":{
						
						"wrist1":{
							"type":"number",
							"minimum": -360,
							"maximum": 360
						}
					}
	    		},

				"forms":[{
					"href":"http://{}/ur10/actions/turnWrist1".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": False,
				"safe": False
			},
            "turnWrist2":{
				"title":"Turn Wrist2",
				"description":"Turn the robot arm Wrist2 joint a certain amount within borders",
				"input":{
					"type":"object",
					"required":["wrist2"],
					"properties":{					
						"wrist2":{
							"type":"number",
							"minimum": -360,
							"maximum": 360
						}
					}
	    		},

				"forms":[{
					"href":"http://{}/ur10/actions/turnWrist2".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": False,
				"safe": False
			},
            "turnWrist3":{
				"title":"Turn Wrist3",
				"description":"Turn the robot arm Wrist3 joint a certain amount within borders",
				"input":{
					"type":"object",
					"required":["wrist3"],
					"properties":{
						
						"wrist3":{
							"type":"number",
							"minimum": -360,
							"maximum": 360
						}
					}
	    		},

				"forms":[{
					"href":"http://{}/ur10/actions/turnWrist3".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": False,
				"safe": False
			},
			"setJointDegrees":{
				"title":"Set the joint positions ",
				"description":"Turn the robot arm joints to a specific degree",
				"input":{
					"type":"object",
					"properties":{
						"base":{
							"type":"number",
							"minimum": -360,
							"maximum": 360
						},
						"shoulder":{
							"type":"number",
							"minimum": -360,
							"maximum": 360
						},
						"elbow":{
							"type":"number",
							"minimum": -360,
							"maximum": 360
						},
						"wrist1":{
							"type":"number",
							"minimum": -360,
							"maximum": 360
						},
						"wrist2":{
							"type":"number",
							"minimum": -360,
							"maximum": 360
						},
						"wrist3":{
							"type":"number",
							"minimum": -360,
							"maximum": 360
						},						
						"async":{
							"type":"boolean",
							"default":False
						},
					}
	    		},
				"forms":[{
					"href":"http://{}/ur10/actions/setjoints".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False

			},

			"goTo":{
				"title":"Go to position",
				"description":"Move to pose with cartesian coordinates x-y-z and rotation vector rx-ry-rz (in radians) with the assigned speed and acceleration, default values for speed and acceleration is 0,5.", 
				"input":{
					"type":"object",
					"properties":{
						"x":{
							"type":"number",
							"minimum": -1300,
							"maximum": 1300
						},
						"y":{
							"type":"number",
							"minimum": -1300,
							"maximum": 1300
						},
						"z":{
							"type":"number",
							"minimum": -1000,
							"maximum": 1000
						},
						"rx":{
							"type":"number",
						},
						"ry":{
							"type":"number",
						},
						"rz":{
							"type":"number",
						},
						"async":{
							"type":"boolean",
							"default":False
						},
						"s": {
							"type": "number",
							"minimum": 0.1,
							"maximum": 1.0
						},
						"a": {
							"type": "number",
							"minimum": 0.1,
							"maximum": 1.0
						}
					}
	    		},
				"forms":[{
					"href":"http://{}/ur10/actions/goto".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False
			},
			"gripClose":{
				"title":"Close Grip",
				"description":"Closes the grip with 120 Newton of force",
				"forms":[{
					"href":"http://{}/ur10/actions/gripClose".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False
			},
			"gripCloseLight":{
				"title":"Close Grip",
				"description":"Closes the grip with 25 Newton of force",
				"forms":[{
					"href":"http://{}/ur10/actions/gripCloseLight".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False
			},
			"gripOpen":{
				"title":"Open Grip",
				"description":"Opens the grip with 120 Newton of force",
				"forms":[{
					"href":"http://{}/ur10/actions/gripOpen".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False
			}
		
		}
			
}