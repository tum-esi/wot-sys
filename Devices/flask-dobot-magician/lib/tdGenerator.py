def getTD(ip_address):
    return {
        "@context": [
            "https://www.w3.org/2019/wot/td/v1",
            {"@language": "en"}
        ],
        'id': 'de:tum:ei:esi:dobot',
        'title': 'Warehouse Dobot',
        'description': "A robot arm that is responsible for getting cubes from the warehouse and retrieving them to the warehouse. The arm can only perform one 'actioninvoke' at a time and any further 'actioninvoke' requests up to a maximum of 3 are queued.",
        "securityDefinitions": {"nosec_sc": {"scheme": "nosec"}},
        "security": "nosec_sc",
        'properties': {
            'position': {
                "title": "The position of the robot arm's end effector.",
                "description": "Get position of the robot arm's end effector relative to its home position. Returns an object containing the linear track positon 'l', as well as x, y, z positions and rotation of the end effector.",
                "type": "object",
                # desc for individual properties and max min would be nice
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"},
                    "r": {"type": "number"},
                    "l": {"type": "number", "minimum": 0, "maximum":900},
                },
                "readOnly": True,
                "forms": [{
                    "href": "http://{}/DobotMagician/properties/position".format(ip_address),
                    "contentType": "application/json",
                    "op": ["readproperty"]
                }]
            }
            
        },
        "actions": {
            # adding additional responses for all the forms. Also sync false
            "calibrateDevice": {
                "title": "Recalibrate the device and then return to the start position in the middle of the rail. This can be invoked if the device does not reach the pre-defined positons accuaretly",
                "description": "Return to the start position, which is in the middle of the linear track.",
                "safe":False,
                "idempotent":True,
                "synchronous": False,
                "forms": [{
                    "href": "http://{}/DobotMagician/actions/calibrateDevice".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "getCube": {
                "title": "Get cube from warehouse queue",
                "description": "Get a cube from the warehouse queue and put it on the second conyevor belt, then pushes the queue. Response is sent as soon as the request is received.",
                "safe":False,
                "idempotent":True,
                "synchronous": False,
                "forms": [{
                    "href": "http://{}/DobotMagician/actions/getCube".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "returnCube": {
                "title": "Return cube to warehouse queue",
                "description": "Return a cube from the first conveyor belt to the warehouse queue. Response is sent as soon as the request is received.",
                "safe":False,
                "idempotent":True,
                "synchronous": False,
                "forms": [{
                    "href": "http://{}/DobotMagician/actions/returnCube".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            }
        }
    }
