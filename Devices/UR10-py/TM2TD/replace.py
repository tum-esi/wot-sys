import json
import _pickle as pickle

with open("TM.json", "r") as f:
    tm_dict = json.load(f)

with open("config.json", "r") as f:
    config = json.load(f)

tm_string = json.dumps(tm_dict)
config_string = json.dumps(config)



config_new = {}

for keyy in config:
    anahtar = r"{{" + str(keyy) + r"}}"
    config_new[anahtar] = config[keyy]

#print(config_new)


for key, value in config_new.items():
    #tm_string = tm_string.replace("{{HTTP_IP_ADDRESS}}", "2222222")
    tm_string = tm_string.replace('"{}"'.format(key), str(value)) #backslah bak
    tm_string = tm_string.replace(key, str(value))


tm_modified_dict = json.loads(tm_string)
#print(tm_modified_dict)


tm_modified_dict["@type"] = "UR-10 Robot Arm"
tm_modified_dict["securityDefinitions"] = {"nosec_sc": {"scheme": "nosec"}}
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
					"htv:methodName": "GET",
					"security": "nosec_sc"
					},
					{"href": "properties/{}".format(key),
					"htv:methodName": "PUT",
                    "op": "writeproperty",
					"contentType":"application/json",
					"security": "nosec_sc"
					}
				]



with open('convertedTD.json', 'w') as file:
     file.write(json.dumps(tm_modified_dict, indent=1, sort_keys=True))   


print(json.dumps(tm_modified_dict, indent=1, sort_keys=True))