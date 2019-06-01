var hueLamp3TD = JSON.stringify({
    "title": "Hue White Lamp 3",
    "id": "urn:dev:ops:32473-HueLight-3",
    "description": "This is a Philips Hue White Light Bulb that can be controlled remotely. It is configured for a specific router.",
    "@context": [
        "https://www.w3.org/2019/wot/td/v1",
        {
            "@language": "en"
        }
    ],
    "securityDefinitions": {
        "nosec_sc": {
            "scheme": "nosec"
        }
    },
    "security": [
        "nosec_sc"
    ],
    "actions": {
        "set_state": {
            "title": "Set State",
            "description": "Allows the user to turn the light on and off, modify the hue and effects",
            "input": {
                "type": "object",
                "properties": {
                    "on": {
                        "description": "On/Off state of the light. On=true, Off=false",
                        "type": "boolean"
                    },
                    "bri": {
                        "description": "Brightness level. Accepts 255 as well",
                        "type": "integer",
                        "minimum": 0,
                        "maximum": 254
                    },
                    "alert": {
                        "description": "The alert effect, which is a temporary change to the bulb’s state.'l' of lselect stands for loop. Presence of lselect ignores transitiontime",
                        "type": "string",
                        "enum": [
                            "lselect",
                            "none",
                            "select"
                        ]
                    },
                    "transisiontime": {
                        "description": "The duration of the transition from the light’s current state to the new state.",
                        "type": "integer",
                        "unit": "100ms",
                        "minimum": 0,
                        "maximum": 65535
                    },
                    "bri_inc": {
                        "description": "Increments or decrements the value of the brightness.  bri_inc is ignored if the bri attribute is provided.",
                        "type": "integer",
                        "minimum": -254,
                        "maximum": 254
                    }
                }
            },
            "output": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "success": {
                            "oneOf": [{
                                    "type": "object",
                                    "properties": {
                                        "/lights/4/state/bri_inc": {
                                            "description": "Increments or decrements the value of the brightness.  bri_inc is ignored if the bri attribute is provided.",
                                            "type": "integer",
                                            "minimum": -254,
                                            "maximum": 254
                                        }
                                    }
                                },
                                {
                                    "type": "object",
                                    "properties": {
                                        "/lights/4/state/on": {
                                            "description": "On/Off state of the light. On=true, Off=false",
                                            "type": "boolean"
                                        }
                                    }
                                },
                                {
                                    "type": "object",
                                    "properties": {
                                        "/lights/4/state/bri": {
                                            "description": "brightness level",
                                            "type": "integer",
                                            "minimum": 0,
                                            "maximum": 254
                                        }
                                    }
                                },
                                {
                                    "type": "object",
                                    "properties": {
                                        "/lights/4/state/alert": {
                                            "description": "The alert effect, which is a temporary change to the bulb’s state.'l' of lselect stands for loop.",
                                            "type": "string",
                                            "enum": [
                                                "lselect",
                                                "none",
                                                "select"
                                            ]
                                        }
                                    }
                                },
                                {
                                    "type": "object",
                                    "properties": {
                                        "/lights/4/state/transisiontime": {
                                            "description": "The duration of the transition from the light’s current state to the new state.",
                                            "type": "integer",
                                            "minimum": 0,
                                            "maximum": 65535
                                        }
                                    }
                                }
                            ]
                        }
                    }
                }
            },
            "forms": [{
                "href": "http://Philips-hue.local/api/R6D7CYQFzXckikMPLEL8WbSZWg9XKkEyx-NrgKws/lights/3/state",
                "contentType": "application/json",
                "htv:methodName": "PUT",
                "op": [
                    "invokeaction"
                ]
            }],
            "idempotent": false,
            "safe": false
        }
    }
});

hueLamp3Thing = WoT.consume(hueLamp3TD);

hueLamp3Thing.actions["set_state"].invoke({
    "on": true,
    "bri": 250
})