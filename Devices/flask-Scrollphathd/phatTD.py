def get_td(ip_address):
    return {
        "@context": [
            "https://www.w3.org/2019/wot/td/v1",
            {"@language": "en"}
        ],
        'id': 'de:tum:ei:esi:phat',
        'title': 'ScrollPhat HD',
        'description': "A scroll-phat-hd that can be remotely controlled.",
        "securityDefinitions": {"nosec_sc": {"scheme": "nosec"}},
        "security": "nosec_sc",
        'properties': {
            'displaySize': {
                "title": "The Display Size",
                "description": "Get the size/shape of the display. Returns a tuple containing the width and height of the display",
                "type": "array",
                "items": {
                    "type": "integer"
                },
                
                "readOnly": True,
                "forms": [{
                    "href": "http://{}/properties/displaySize".format(ip_address),
                    "contentType": "application/json",
                    "op": ["readproperty"]
                }]
            }
            
        },
        "actions": {
            "setPixel": {
                "title": "Turn on Pixel",
                "description": "Light a specific single pixel with a given brightness. x and y are 5 by default.",
                "safe":False,
                "idempotent":True,
                "input": {
                    "type": "object",
                    "required": ["brightness"],
                    "properties": {
                        "x": {
                            "type": "integer",
                            "default": 5,
                            "minimum": 0,
                            "maximum": 16
                        },
                        "y": {
                            "type": "integer",
                            "default": 5,
                            "minimum": 0,
                            "maximum": 6
                        },
                        "brightness": {
                            "type": "number",
                            "minimum": 0.0,
                            "maximum": 1.0    
                        }
                    },
                },
                "forms": [{
                    "href": "http://{}/actions/setPixel".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "writeString": {
                "title": "Display String",
                "description": "Write a string to the screen. Calls draw_char for each character.",
                "safe":False,
                "idempotent":True,
                "input": {
                    "type": "object",
                    "required": ["string","x", "y", "brightness"],
                    "properties": {
                        "string": {
                            "description": "The string to display.",
                            "type": "string",
                            "maxLength": 50
                        },
                        "time":{
                            "description": "duration the string keep scrolling, it is 5 seconds by default. Adjust the duration according to the lenght of your string.",
                            "default": 5,
                            "type":"integer",
                            "minimum": 3,
                            "maximum": 30
                        },
                        "x": {
                            "description": "Offset x - distance of the string from the left of the screen",
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 17
                        },
                        "y": {
                            "description": "Offset x - distance of the string from the left of the screen",
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 7
                        },
                        "brightness": {
                            "type": "number",
                            "minimum": 0.0,
                            "maximum": 1.0    
                        }
                    },
                },
                "forms": [{
                    "href": "http://{}/actions/writeString".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "writeChar": {
                "title": "Display Character",
                "description": "Write a single char to the screen. Returns the x and y coordinates of the bottom left-most corner of the drawn character.Shows the char for 5 secs",
                "safe":False,
                "idempotent":True,
                "input": {
                    "type": "object",
                    "required": ["char", "brightness"],
                    "properties": {
                        "char": {
                            "description": "Char to display- either an integer ordinal or a single letter",
                            "type": "string",
                            "minLength": 1,
                            "maxLength": 1
                        },
                        "o_x": {
                            "description": "Offset x - distance of the string from the left of the screen. By default its value is 5",
                            "type": "integer",
                            "default": 5,
                            "minimum": 0,
                            "maximum": 17
                        },
                        "o_y": {
                            "description": "Offset x - distance of the string from the left of the screen. By default its value is 0",
                            "type": "integer",
                            "default": 0,
                            "minimum": 0,
                            "maximum": 7
                        },
                        "brightness": {
                            "type": "number",
                            "minimum": 0.0,
                            "maximum": 1.0    
                        }
                    },
                },
                "forms": [{
                    "href": "http://{}/actions/writeChar".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },

            "fill": {
                "title": "Fill the Screen",
                "description": "Fill an area of the display.",
                "safe": False,
                "idempotent": True,
                "input": {
                    "type": "object",
                    "required": ["brightness"],
                    "properties": {
                        "x": {
                            "description": "Offset x - distance of the area from the left of the screen. 0 by default",
                            "type": "integer",
                            "default": 0,
                            "minimum": 0,
                            "maximum": 17
                        },
                        "y": {
                            "description": "Offset y - distance of the area from the left of the screen. 0 by default",
                            "type": "integer",
                            "default": 0,
                            "minimum": 0,
                            "maximum": 7
                        },
                        "brightness": {
                            "type": "number",
                            "minimum": 0.0,
                            "maximum": 1.0    
                        },
                        "width": {
                            "description": "Width of the area. 17 by default",
                            "type": "integer",
                            "default": 17,
                            "minimum": 0,
                            "maximum": 17
                        },
                        "height": {
                            "description": "Height of the area. 7 by default",
                            "type": "integer",
                            "default": 7,
                            "minimum": 0,
                            "maximum": 7
                        }
                     },
                },
                "forms": [{
                    "href": "http://{}/actions/fill".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "clearRect": {
                "title": "Clear Screen Area",
                "description": "Clear a specified rectangular area of the screen.",
                "safe": False,
                "idempotent": True,
                "input": {
                    "type": "object",
                    "required": ["x", "y"],
                    "properties": {
                        "x": {
                            "description": "Offset x - distance of the area from the left of the screen. 0 by default",
                            "type": "integer",
                            "default": 0,
                            "minimum": 0,
                            "maximum": 17
                        },
                        "y": {
                            "description": "Offset y - distance of the area from the left of the screen. 0 by default",
                            "type": "integer",
                            "default": 0,
                            "minimum": 0,
                            "maximum": 7
                        },
                        "width": {
                            "description": "Width of the area. 17 by default",
                            "type": "integer",
                            "default": 17,
                            "minimum": 0,
                            "maximum": 17
                        },
                        "height": {
                            "description": "Height of the area. 7 by default",
                            "type": "integer",
                            "default": 7,
                            "minimum": 0,
                            "maximum": 7
                        }
                     },
                },
                "forms": [{
                    "href": "http://{}/actions/clearRect".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            
            "clear": {
                "title": "Clear the entire screen.",
                "description": "clears all pixels.",
                "safe": False,
                "idempotent": True,
                "forms": [{
                    "href": "http://{}/actions/clear".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "scroll" : {
                "title": "Shift Image",
                "description": "Scroll pHAT HD displays an 17x7 pixel window into the bufer, which starts at the left offset and wraps around. The x and y values are added to the internal scroll offset.",
                "input" : {
                    "type": "object",
                    "properties": {
                        "x": {
                            "description": "Amount to scroll on x-axis. (default 1)",
                            "type": "integer",
                            "default": 1
                            
                        },
                        "y": {
                            "description": "Amount to scroll on y-axis. (default 0)",
                            "type": "integer",
                            "default": 0
                        }
                    }
                },
                "forms": [{
                    "href": "http://{}/actions/scroll".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]

            },

            "showPulse":{
                "title": "Display Pulse",
                "description": "shows a pulse graph on screen for 10 seconds",
                "forms": [{
                    "href": "http://{}/actions/showPulse".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },
            "clock":{
                "title": "Show Time",
                "description": "shows current time on screen for 10 seconds",
                "forms": [{
                    "href": "http://{}/actions/clock".format(ip_address),
                    "contentType": "application/json",
                    "op": "invokeaction"
                }]
            },


            "sendImage": {
                "title": "Upload Image",
                "description": "Takes a bmp image with size 17-7 pixels with bpp value of 8 as input, and displays it on the screen.",
                "safe":False,
                "idempotent":True,
                "input": {
  
                },
                "forms": [{
                    "description": "The payload is multipart/form-data which should be indicated in curl command with '--form' flag as in: curl --location --request POST 'http://<device ip>:8080/actions/sendImage' --form 'example.bmp=@'<directory of image>/example.bmp''",
                    "href": "http://{}/actions/sendImage".format(ip_address),
                    "contentType": "image/bmp",
                    "op": "invokeaction"
                }]
            }
           
        }
    }

