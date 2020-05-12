from Robot import *
from HTTP import *
from MQTT import *
from Uarm_TD import get_td
import time
import requests
import _thread


time.sleep(10)

TD_Directory_Address= "http://192.168.0.100:8080"
#broker IP address
MQTT_ip_address= "192.168.0.116:1883"
#Uarms IP adress
HTTP_ip_address= "172.16.1.209:8080"

TD_Uarm = get_td( HTTP_ip_address,MQTT_ip_address)

push_HTTP_TD(TD_Uarm)


def submit_td(ip_addr,td):
	print("Uploading TD to directory ...")
	while True:
		try:
			r = requests.post("{}/api/td".format(ip_addr),json=td)
			r.close()
			print("Got response: ", r.status_code)
			if 200 <= r.status_code <= 299:
				print("TD uploaded")
				return
		except Exception as e:
			print(e)
			print("TD could not be uploaded. Will try again in 15 Seconds ...")
			time.sleep(15)
			

#starts Gripper ROS node
threading.Thread(target=lambda: rospy.init_node('gripper_node', disable_signals=True)).start()
#not working right now bc database not connected
#_thread.start_new_thread(submit_td, (TD_Directory_Address,TD_Uarm))


#starts MQTT (not working bc no mqtt broker in system)
#main_MQTT(TD_Uarm)

#starts HTTP flask server
app.run(host=os.environ['ROS_IP'], port=8080)


