def get_td(HTTP_ip_address, MQTT_ip_address):
	return{
		"@context": "https://www.w3.org/2019/wot/td/v1",
	    "id": "de.tum:ei:esi:uArm:192.168.0.112:8080",
	    "title": "Uarm",
	    "securityDefinitions": { "nosec_sc": {"scheme": "nosec"}
	    },
	    "security": ["nosec_sc"],
	    "properties": {
	    	"homeLoc":{
	    		"title":"Home location",
	    		"description":"Home location, can be set to a new value. goHome action goes there",
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
				"forms": [
					{
					"href": "http://{}/uarm/properties/homeloc".format(HTTP_ip_address),
					"op": ["readproperty","writeproperty"],
					"contentType":"application/json"
					},
					{
					"href":"mqtt://{}/uarm/properties/homeloc".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":["readproperty","observeproperty"],
					"mqv:controlPacketValue": "SUBSCRIBE"
					},
					{
					"href":"mqtt://{}/uarm/properties/homeloc/writeproperty".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":"writeproperty",
					"mqv:controlPacketValue": "PUBLISH"
					}
				]
	    	},
			"location":{
	    		"title":"Return location",
	    		"description":"Gives back the current location",
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
				"forms": [
					{"href": "http://{}/uarm/properties/location".format(HTTP_ip_address),
					"op": ["readproperty"],
					"contentType":"application/json"
					},
					{
					"href":"mqtt://{}/uarm/properties/location".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":["readproperty","observeproperty"],
					"mqv:controlPacketValue": "SUBSCRIBE"
					}
				]
	    	}
	    },
	
	    "actions":{
			"beep":{
				"title":"Beep",
				"description":"Beep for 1 sec",
				"forms":[{
					"href":"http://{}/uarm/actions/beep".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				},
				{
					"href":"mqtt://{}/uarm/actions/beep".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"mqv:controlPacketValue": "PUBLISH"
				}],
				"idempotent": True,
				"safe": False
			},
			
			"beepWithTime":{
				"title":"Beep with Time set",
				"description":"Beep for a time set between 1-3 sec",
				"input":{
					"type":"integer",
					"minimum": 1,
					"maximum": 3
				},
				"forms":[
					{
					"href":"http://{}/uarm/actions/beepwithtime".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
					},
					{
					"href":"mqtt://{}/uarm/actions/beepwithtime".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"mqv:commandCode":3
				}],
				"idempotent": True,
				"safe": False
			},
			"goHome":{
				"title":"Go Home",
				"description":"Go to the set home point",
				"forms":[{
					"href":"http://{}/uarm/actions/gohome".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				},
				{
					"href":"mqtt://{}/uarm/actions/gohome".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"mqv:controlPacketValue": "PUBLISH"
				}],
				"idempotent": False,
				"safe": False
			},
			
			"turnLeft":{
				"title":"Turn Left",
				"description":"Turn robot arm to left for 1 step",
				"input":{
					"type":"object",
					"required":["y"],
					"properties":{
						
						"y":{
							"type":"number",
							"minimum": -200,
							"maximum": 200
						}
					}
	    		},
				"forms":[{
					"href":"http://{}/uarm/actions/turnleft".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				},
				{
					"href":"mqtt://{}/uarm/actions/turnleft".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"mqv:controlPacketValue": "PUBLISH"
				}],
				"idempotent": False,
				"safe": False
			},
			"turnRight":{
				"title":"Turn Right",
				"description":"Turn robot arm to right for a certain distance (has yet to be implemented in loT.py)",
				"input":{
					"type":"object",
					"required":["y"],
					"properties":{
						
						"y":{
							"type":"number",
							"minimum": -200,
							"maximum": 200
						}
					}
	    		},
				"forms":[{
					"href":"http://{}/uarm/actions/turnright".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				},
				{
					"href":"mqtt://{}/uarm/actions/turnright".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"mqv:controlPacketValue": "PUBLISH"
				}],
				"idempotent": False,
				"safe": False
			},
			"goTo":{
				"title":"Go to position",
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
							"type":"number",
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
					"href":"http://{}/uarm/actions/goto".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				},
				{
					"href":"mqtt://{}/uarm/actions/goto".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"mqv:controlPacketValue": "PUBLISH"
				}],
				"idempotent": True,
				"safe": False
			},
			"goWithSpeed":{
				"title":"Go to with speed",
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
					"href":"http://{}/uarm/actions/gowithspeed".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				},
				{
					"href":"mqtt://{}/uarm/actions/gowithspeed".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"mqv:controlPacketValue": "PUBLISH"
				}],
				"idempotent": True,
				"safe": False
			},
			"sequence1":{
				"title":"Grip Routine",
				"description":"Routine to move to a position and close the grip",
				"forms":[{
					"href":"http://{}/uarm/actions/sequence1".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				},
				{
					"href":"mqtt://{}/uarm/actions/sequence1".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"mqv:controlPacketValue": "PUBLISH"
				}],
				"idempotent": True,
				"safe": False
			},
			"gripClose":{
				"title":"Close Grip",
				"description":"Closes the grip",
				"forms":[{
					"href":"http://{}/uarm/actions/gripclose".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				},
				{
					"href":"mqtt://{}/uarm/actions/gripclose".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"mqv:controlPacketValue": "PUBLISH"
				}],
				"idempotent": True,
				"safe": False
			},
			"gripOpen":{
				"title":"Open Grip",
				"description":"Opens the grip",
				"forms":[{
					"href":"http://{}/uarm/actions/gripopen".format(HTTP_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				},
				{
					"href":"mqtt://{}/uarm/actions/gripopen".format(MQTT_ip_address),
					"contentType":"application/json",
					"op":"invokeaction",
					"mqv:controlPacketValue": "PUBLISH"
				}],
				"idempotent": True,
				"safe": False
			}
		
		}
			
}