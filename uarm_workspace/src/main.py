from Robot import *
from lot import *
from MQTT import *

#starts Gripper ROS node
threading.Thread(target=lambda: rospy.init_node('gripper_node', disable_signals=True)).start()
#starts MQTT
main_MQTT()
#starts HTTP flask server
app.run(host=os.environ['ROS_IP'], port=8080)


