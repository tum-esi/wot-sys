def get_td(ip_address):
    return {
        "@context": [
            "https://www.w3.org/2019/wot/td/v1",
            {"@language": "en"}
        ],
        'id': 'de:tum:ei:esi:flora',
        'title': 'Flora Color Sensor',
        'description': "A color sensor with a white LED",
        "securityDefinitions": {"nosec_sc": {"scheme": "nosec"}},
        "security": "nosec_sc",
        'properties': {
            'color': {
                "title": "Detected Color",
                "description": "Returns the calculated color value as R,G,B values. Higher the individual, more likely that color",
                "type": "array",
                "items":{
                    "type":"number",
                    "minimum":0,
                    "maximum":255
                },
                "readOnly": True,
                "forms": [{
                    "href": "http://{}/properties/color".format(ip_address),
                    "contentType": "application/json",
                    "op": "readproperty"
                }]
            },
            'temperature': {
                "title": "Color Temperature",
                "description": "Returns raw color temperature that can be used to deduce color value when combined with illuminance",
                "type": "number",
                "minimum":1000,
                "maximum":20000,
                "readOnly": True,
                "forms": [{
                    "href": "http://{}/properties/temperature".format(ip_address),
                    "contentType": "application/json",
                    "op": "readproperty"
                }]
            },
            'lux': {
                "title": "Illuminance",
                "description": "Returns raw illuminance value that can be used to deduce color value when combined with color temperature",
                "type": "number",
                "minimum":100,
                "maximum":14000,
                "readOnly": True,
                "forms": [{
                    "href": "http://{}/properties/lux".format(ip_address),
                    "contentType": "application/json",
                    "op": "readproperty"
                }]
            }
            
        }
    }

