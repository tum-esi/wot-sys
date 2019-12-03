from Robot import *
from HTTP import *
from MQTT import *
from Uarm_TD import get_td
import time
import requests
import _thread

time.sleep(10)

TD_Directory_Address= "http://192.168.0.100:8080"

ip_address_MQTT= "192.168.0.116:1883"
TD_MQTT=get_td(ip_address_MQTT)

ip_address_HTTP= "192.168.0.112:8080"
TD_HTTP=get_td(ip_address_HTTP)

push_HTTP_TD(TD_HTTP)


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

_thread.start_new_thread(submit_td, (TD_Directory_Address,TD_HTTP))

#starts MQTT
main_MQTT(TD_MQTT)
#starts HTTP flask server
app.run(host=os.environ['ROS_IP'], port=8080)


