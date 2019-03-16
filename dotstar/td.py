def get_td(ip_address, leds):
    return {
        'id': 'de:tum:ei:esi:dotstar:{}'.format(ip_address),
        'name': 'DotStar RGB LED strip',
        'description': 'A strip of {} RGB LEDs that can be controlled remotely.'.format(leds),
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
                    "contentType": "application/json"
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
                    "beightness": {
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
                    "contentType": "application/json"
                }]
            }
        },
        "actions": {
            "dot": {
                "description": "Light a specific LED with the given RGB color.",
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
                    "contentType": "application/json"
                }]
            },
            "fill": {
                "description": "Light all the LEDs with same color.",
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
                    "contentType": "application/json"
                }]
            },
            "fill_upper": {
                "description": "Light all the LEDs in the upper band with the same color.",
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
                    "contentType": "application/json"
                }]
            },
            "fill_middle": {
                "description": "Light all the LEDs in the middle band with the same color.",
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
                    "contentType": "application/json"
                }]
            },
            "fill_lower": {
                "description": "Light all the LEDs in the lower band with the same color.",
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
                    "contentType": "application/json"
                }]
            },
            "random": {
                "description": "Light up all LEDs with random colors.",
                "forms": [{
                    "href": "http://{}/actions/random".format(ip_address),
                    "contentType": "application/json"
                }]
            },
            "shutdown": {
                "description": "Turn off all the LEDs.",
                "forms": [{
                    "href": "http://{}/actions/shutdown".format(ip_address),
                    "contentType": "application/json"
                }]
            }
        }
    }
