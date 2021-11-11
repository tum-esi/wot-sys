import json
import _pickle as pickle

with open("TM.json", "r") as f:
    tm_dict = json.load(f)

with open("config.json", "r") as f:
    config = json.load(f)

tm_string = json.dumps(tm_dict)
config_string = json.dumps(config)


#creating a new dict using the keys and values from the json object in config.json and adding brackets on each side of each key. Later on the updated key strings in TM.json are changed with the corresponding value in the new_config dict. 
config_for_TM = {}

for key_old in config:
    key_new = r"{{" + str(key_old) + r"}}"
    config_for_TM[key_new] = config[key_old]



for key, value in config_for_TM.items():
    tm_string = tm_string.replace('"{}"'.format(key), str(value))
    tm_string = tm_string.replace(key, str(value))


tm_modified_dict = json.loads(tm_string)

tm_modified_dict["@type"] = config["@type"]
tm_modified_dict["securityDefinitions"] = config["securityDefinitions"]
tm_modified_dict["security"] = ["nosec_sc"]
tm_modified_dict["links"]= [{
		"rel" : "type",
		"href" : "",
		"type": "application/td+json"
	}]


for key in tm_modified_dict["actions"]:
    tm_modified_dict["actions"][key]["forms"]= [{
					"href":"actions/{}".format(key),
					"contentType":"application/json",
					"op":"invokeaction",
					"htv:methodName":"POST"
				}],

for key in tm_modified_dict["properties"]:
    if tm_modified_dict["properties"][key]["readOnly"] == True:
        tm_modified_dict["properties"][key]["forms"]= [
					{"href": "properties/{}".format(key),
					"op": ["readproperty"],
					"contentType":"application/json"
					}
				]

    elif tm_modified_dict["properties"][key]["readOnly"] == False and tm_modified_dict["properties"][key]["writeOnly"] == False:
        tm_modified_dict["properties"][key]["forms"]= [
					{"href": "properties/{}".format(key),
					"op": "readproperty",
					"contentType":"application/json",
					"htv:methodName": "GET"
					},
					{"href": "properties/{}".format(key),
					"htv:methodName": "PUT",
                    "op": "writeproperty",
					"contentType":"application/json"
					
					}
				]

with open('convertedTD.json', 'w') as file:
     file.write(json.dumps(tm_modified_dict, indent=1, sort_keys=True))   


