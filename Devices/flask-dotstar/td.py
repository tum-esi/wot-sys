def get_td(ip_address, leds):
    return {
        "@context": [
            "https://www.w3.org/2019/wot/td/v1",
            {"@language": "en"}
        ],
        'id': 'de:tum:ei:esi:dotstar:{}'.format(ip_address),
        'title': 'DotStar RGB LED strip',
        'description': 'A strip of {} RGB LEDs that can be controlled remotely.'.format(leds),
        "securityDefinitions": {"nosec_sc": {"scheme": "nosec"}},
        "security": "nosec_sc",
        'properties': {
            'brightness': {
                "title": "LED Brightness",
                "description": "Set all LEDs to the same brightness between 0 and 100%.",
                "type": "integer",
                "minimum": 0,
                "maximum": 100,
                "unit": "%",
                "readOnly": False,
                "writeOnly": False,
                "forms": [{
                    "href": "http://{}/properties/brightness".format(ip_address),
                    "contentType": "application/json",
                    "op": ["readproperty", "writeproperty"]
                }]
            },
            'stats': {
                "title": "LED Stats",
                "description": "Get detailed information about the current state of the LED strip.",
                "type": "object",
                "properties": {
                    "nr_of_leds": {
                        "type": "integer"
                    },
                    "nr_of_leds_on": {
                        "type": "integer",
                        "minimum": 0,
                        "maximum": leds
                    },
                    "brightness": {
                        "type": "integer",
                        "minimum": 0,
                        "maximum": 100,
                        "unit": "%"
                    },
                    "led_colors": {
                        "type": "array",
                        "items": {
                            "type": "array",
                            "items": [
                                {
                                    "type": "integer",
                                    "minimum": 0,
                                    "maximum": 255
                                },
                                {
                                    "type": "integer",
                                    "minimum": 0,
                                    "maximum": 255
                                },
                                {
                                    "type": "integer",
                                    "minimum": 0,
                                    "maximum": 255
                                }
                            ]
                        }
                    }
                },
                "readOnly": True,
                "writeOnly": False,
                "forms": [{
                    "href": "http://{}/properties/stats".format(ip_address),
                    "contentType": "application/json",
                    "op": "readproperty"
                }]
            }
        },
        "actions": {
            "dot": {
                "description": "Light a specific single LED with the given RGB color.",
                "safe":False,
                "idempotent":True,
                "input": {
                    "type": "object",
                    "required": ["led", "color"],
                    "properties": {
                        "led": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": leds
                        },
                        "color": {
                            "type": "object",
                            "required": ["red", "green", "blue"],
                            "properties": {
                                "red": {
                                    "type": "integer",
                                    "minimum": 0,
                                    "maximum": 255
                                },
                                "green": {
                                    "type": "integer",
                                    "minimum": 0,
                                    "maximum": 255
                                },
                                "blue": {
                                    "type": "integer",
                                    "minimum": 0,
                                    "maximum": 255
                                }
                            }
                        }
                    },
                },
                "forms": [{
                    "href": "http://{}/actions/dot".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "fill_array": {
                "description": "Light the LEDs in between given numbers with same color.",
                "safe": False,
                "idempotent": True,
                "type": "object",
                "input": {
                    "type": "object",
                    "required": ["ledBegin","ledEnd", "color"],
                    "properties": {
                        "ledBegin": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": leds
                        },
			            "ledEnd": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": leds
                        },
                        "color": {
                            "type": "object",
                            "required": ["red", "green", "blue"],
                            "properties": {
                                "red": {
                                    "type": "integer",
                                    "minimum": 0,
                                    "maximum": 255
                                },
                                "green": {
                                    "type": "integer",
                                    "minimum": 0,
                                    "maximum": 255
                                },
                                "blue": {
                                    "type": "integer",
                                    "minimum": 0,
                                    "maximum": 255
                                }
                            }
                        }
                     },
                },
                "forms": [{
                    "href": "http://{}/actions/fill_array".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "fill": {
                "description": "Light all the LEDs with same color.",
                "safe": False,
                "idempotent": True,
                "input": {
                    "type": "object",
                    "required": ["red", "green", "blue"],
                    "properties": {
                        "red": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 255
                        },
                        "green": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 255
                        },
                        "blue": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 255
                        }
                    },
                },
                "forms": [{
                    "href": "http://{}/actions/fill".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "fill_upper": {
                "description": "Light all the LEDs in the upper band with the same color.",
                "safe": False,
                "idempotent": True,
                "input": {
                    "type": "object",
                    "required": ["red", "green", "blue"],
                    "properties": {
                        "red": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 255
                        },
                        "green": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 255
                        },
                        "blue": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 255
                        }
                    },
                },
                "forms": [{
                    "href": "http://{}/actions/fill_upper".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "fill_middle": {
                "description": "Light all the LEDs in the middle band with the same color.",
                "safe": False,
                "idempotent": True,
                "input": {
                    "type": "object",
                    "required": ["red", "green", "blue"],
                    "properties": {
                        "red": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 255
                        },
                        "green": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 255
                        },
                        "blue": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 255
                        }
                    },
                },
                "forms": [{
                    "href": "http://{}/actions/fill_middle".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "fill_lower": {
                "description": "Light all the LEDs in the lower band with the same color.",
                "safe": False,
                "idempotent": True,
                "input": {
                    "type": "object",
                    "required": ["red", "green", "blue"],
                    "properties": {
                        "red": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 255
                        },
                        "green": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 255
                        },
                        "blue": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 255
                        }
                    },
                },
                "forms": [{
                    "href": "http://{}/actions/fill_lower".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "random": {
                "description": "Light up all LEDs with random colors.",
                "safe": False,
                "idempotent": False,
                "forms": [{
                    "href": "http://{}/actions/random".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "shutdown": {
                "description": "Turn off all the LEDs.",
                "safe": False,
                "idempotent": True,
                "forms": [{
                    "href": "http://{}/actions/shutdown".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            }
        }
    }