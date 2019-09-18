def get_td(ip_address):
	return{
		"@context": "https://www.w3.org/2019/wot/td/v1",
	    "id": "de.tum:ei:esi:uArm:192.168.0.112:8080",
	    "title": "Uarm",
	    "securityDefinitions": { "nosec_sc": {"scheme": "nosec"}
	    },
	    "security": ["nosec_sc"],
	    "properties": {
	    	"homeloc":{
	    		"title":"Home location",
	    		"description":"Sets the new Home location",
	    		"type":"object",
	    		"required":["x","y","z"],
	    		"properties":{
	    			"x":{
						"type":"integer",
						"minimum": 120,
						"maximum": 200
						},
	    			"y":{
						"type":"integer",
						"minimum": -200,
						"maximum": 200
						},
	    			"z":{
						"type":"integer",
						"minimum": 0,
						"maximum": 60
						}
	    		},
	    		"forms": [{"href": "http://{}/properties/homeloc".format(ip_address),
				"op": ["readproperty","writeproperty"],
				"contentType":"application/json"
				}]
	    	}
	    },
	    "actions":{
	
			"beep":{
				"title":"Beep",
				"description":"Beep for 1 sec",
				"forms":[{
					"href":"http://{}/actions/beep".format(ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False
			},
			
			"beepwithtime":{
				"title":"Beep with Time set",
				"description":"Beep for a time set between 1-3 sec",
				"input":{
					"type":"integer",
					"minimum": 0,
					"maximum": 3
				},
				"forms":[{
					"href":"http://{}/actions/beepwithtime".format(ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False
			},
			"gohome":{
				"title":"Go Home",
				"description":"Go to the set home point",
				"forms":[{
					"href":"http://{}/actions/gohome".format(ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": False,
				"safe": False
			},
			
			"turnleft":{
				"title":"Turn left",
				"description":"Turn robot arm to left for a certain distance (  yet to be implemented in loT.py)",
				"forms":[{
					"href":"http://{}/actions/turnleft".format(ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": False,
				"safe": False
			},
			"turnright":{
				"title":"Turn right",
				"description":"Turn robot arm to right for a certain distance (has yet to be implemented in loT.py)",
				"forms":[{
					"href":"http://{}/actions/turnright".format(ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": False,
				"safe": False
			},
			"goto":{
				"title":"Go to Position",
				"description":"Move to Position x,y,z given by the user with fixed speed=3000",
				"input":{
					"type":"object",
					"required":["x","y","z"],
					"properties":{
						"x":{
							"type":"integer",
							"minimum": 120,
							"maximum": 200
						},
						"y":{
							"type":"integer",
							"minimum": -200,
							"maximum": 200
						},
						"z":{
							"type":"integer",
							"minimum": 0,
							"maximum": 60
						}
					}
	    		},
				"forms":[{
					"href":"http://{}/actions/goto".format(ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False
			},
			"gowithspeed":{
				"title":"Go to Position x,y,z with set speed",
				"description":"Move to described Position with set speed",
				"input":{
					"type":"object",
					"required":["x","y","z","speed"],
					"properties":{
						"x":{
							"type":"integer",
							"minimum": 120,
							"maximum": 200
						},
						"y":{
							"type":"integer",
							"minimum": -200,
							"maximum": 200
						},
						"z":{
							"type":"integer",
							"minimum": 0,
							"maximum": 60
						},
						"speed":{
							"type":"integer",
							"minimum": 100,
							"maximum": 10000
						}
					}
	    		},
				"forms":[{
					"href":"http://{}/actions/gowithspeed".format(ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False
			},
			"grip":{
				"title":"Grip Routine",
				"description":"Routine to move to a position and close the grip",
				"forms":[{
					"href":"http://{}/actions/grip".format(ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False
			},
			"gripclose":{
				"title":"Close Grip",
				"description":"Closes the grip",
				"forms":[{
					"href":"http://{}/actions/gripclose".format(ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False
			},
			"gripopen":{
				"title":"Open Grip",
				"description":"Opens the grip",
				"forms":[{
					"href":"http://{}/actions/gripopen".format(ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False
			},
			
			"gripanddrop":{
				"title":"Grip and Drop Routine",
				"description":"Routine to close grip at a fixed point and drag to another to open the grip",
				"forms":[{
					"href":"http://{}/actions/gripanddrop".format(ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],
				"idempotent": True,
				"safe": False
			}
		
		}
		
		
}
