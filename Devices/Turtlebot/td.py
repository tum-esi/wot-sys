def get_td(ip_address):
	return {
			"@context": "https://www.w3.org/2019/wot/td/v1",
					"id": "de.tum:ei:esi:turtlebot3:1",
					"title": "Turtlebot3",
					"description": "A robotic arm on wheels with full control on wheel movement and predefined positions for the arm",
					"securityDefinitions": { "nosec_sc": {"scheme": "nosec"}},
					"security": ["nosec_sc"],
					"actions":{
				
						"baseForward":{
							"title":"Move Base Forward",
							"description":"Move forward for 1 second",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/baseforward".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"baseForwardWithTime":{
							"title":"Move Base Forward with a set time",
							"description":"Move forward for set second",
							"input":{
								"type": "integer"
							},
							"forms":[{
								"href":"http://{}/turtlebot3/actions/baseforwardwithtime".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"baseBackward":{
							"title":"Move Base Backward",
							"description":"Move backward for 1 second",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/basebackward".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"baseBackwardWithTime":{
							"title":"Move Base Backward with a set time",
							"description":"Move backward for 1 second",
							"input":{
								"type": "integer"
							},
							"forms":[{
								"href":"http://{}/turtlebot3/actions/basebackwardwithtime".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"baseRotateLeft":{
							"title":"Rotate Base Left",
							"description":"Starts the rotation of the base to the left, can be stacked to increase speed",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/baserotateleft".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"baseRotateLeftWithTime":{
							"title":"Rotate Base Left with Time",
							"description":"Rotates to the left for set second",
							"input":{
								"type": "integer"
							},
							"forms":[{
								"href":"http://{}/turtlebot3/actions/baserotateleftwithtime".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"rotateRight":{
							"title":"Rotate Base Right",
							"description":"Starts the rotation of the base to the right, can be stacked to increase speed",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/baserotateright".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"rotateRightWithTime":{
							"title":"Rotate Base Right with Time",
							"description":"Rotates to the right for set second",
							"input":{
								"type": "integer"
							},
							"forms":[{
								"href":"http://{}/turtlebot3/actions/baserotaterightwithtime".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"baseStop":{
							"title":"Stop Base Movement",
							"description":"Stop Base Movement",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/basestop".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"moveArmPosition1":{
							"title":"Move arm to position 1",
							"description":"Arm moves to the middle of the base to a high position",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/armpose1".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"moveArmPosition2":{
							"title":"Move arm to position 2",
							"description":"Arm moves to the middle of the base to a mid level position",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/armpose2".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"moveArmPosition3":{
							"title":"Move arm to position 3",
							"description":"Arm moves to the middle of the base to a low position",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/armpose3".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"moveArmPosition4":{
							"title":"Move arm to position 4",
							"description":"Arm moves to the left of the base to a high position",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/armpose4".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"moveArmPosition5":{
							"title":"Move arm to position 5",
							"description":"Arm moves to the left of the base to a mid level position",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/armpose5".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"moveArmPosition6":{
							"title":"Move arm to position 6",
							"description":"Arm moves to the left of the base to a low position",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/armpose6".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"moveArmPosition7":{
							"title":"Move arm to position 7",
							"description":"Arm moves to the right of the base to a high position",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/armpose7".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"moveArmPosition8":{
							"title":"Move arm to position 8",
							"description":"Arm moves to the right of the base to a mid level position",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/armpose8".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"moveArmPosition9":{
							"title":"Move arm to position 9",
							"description":"Arm moves to the right of the base to a low position",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/armpose9".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": False,
							"safe": False
						},
						"closeGrip":{
							"title":"Close the Gripper",
							"description":"Closes the Gripper",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/gripclose".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": True,
							"safe": False
						},
						"openGrip":{
							"title":"Open the Gripper",
							"description":"Opens the Gripper",
							"forms":[{
								"href":"http://{}/turtlebot3/actions/gripopen".format(ip_address),
								"contentType":"application/json",
								"op":"invokeaction",
								"htv:methodName":"POST"
							}],
							"idempotent": True,
							"safe": False
						}
					}
			}


