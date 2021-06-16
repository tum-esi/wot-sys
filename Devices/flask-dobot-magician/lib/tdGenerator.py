def getTD(ip_address):
    return {
        "@context": [
            "https://www.w3.org/2019/wot/td/v1",
            {"@language": "en"}
        ],
        'id': 'de:tum:ei:esi:dobot',
        'title': 'Warehouse Dobot',
        'description': "A robot arm that is responsible for getting cubes from the warehouse and retrieving them to the warehouse.",
        "securityDefinitions": {"nosec_sc": {"scheme": "nosec"}},
        "security": "nosec_sc",
        'properties': {
            'getPosition': {
                "title": "The position of the robot arm's end effector.",
                "description": "Get position of the robot arm's end effector relative to its home position. Returns an object containing the linear track positon 'l', as well as x, y, z positions and rotation of the end effector.",
                "type": "object",
                # desc for individual properties and max min would be nice
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"},
                    "r": {"type": "number"},
                    "l": {"type": "number", "min": "0", "max":"900"},
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
            "returnToStartPosition": {
                "title": "Go to start postion",
                "description": "Return to the start position, which is in the middle of the linear track.",
                "safe":False,
                "idempotent":True,
                "synchronous": False,
                "forms": [{
                    "href": "http://{}/DobotMagician/actions/returnToStartPosition".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "getCube": {
                "title": "Get cube from warehouse queue",
                "description": "Get a cube from the warehouse queue and put it on the first conyevor belt, then pushes the queue. Response is sent when the the cube is dropped on the conyevor belt.",
                "safe":False,
                "idempotent":True,
                "forms": [{
                    "href": "http://{}/DobotMagician/actions/getCube".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "returnCube": {
                "title": "Return cube to warehouse queue",
                "description": "Return a cube from the second conveyor belt to the warehouse queue. Response is sent when the cube is dropped in the warehouse.",
                "safe":False,
                "idempotent":True,
                "forms": [{
                    "href": "http://{}/DobotMagician/actions/returnCube".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            }
        }
    }
