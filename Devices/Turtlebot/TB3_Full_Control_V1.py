from TB3_Robot import *
import TB3_Robot
from flask import Flask, request, json, jsonify, abort
import threading
import rospy

#Initialize Flask
app = Flask(__name__)

@app.route('/turtlebot3/actions/baseforward', methods= ["POST"])
def forward():
    base_move_forward()
    return ("",204)

@app.route('/turtlebot3/actions/baseforwardwithtime', methods= ["POST"])
def forwardwithtime():
    if request.is_json:
        seconds = request.json
        base_move_forward_with_time(seconds)
        return ("",204)
    else:
        abort(415)

@app.route('/turtlebot3/actions/basebackward', methods= ["POST"])
def backward():
    base_move_backward()
    return ("",204)

@app.route('/turtlebot3/actions/basebackwardwithtime', methods= ["POST"])
def backwardwithtime():
    if request.is_json:
        seconds = request.json
        base_move_backward_with_time(seconds)
        return ("",204)
    else:
        abort(415)

@app.route('/turtlebot3/actions/baserotateleft', methods= ["POST"])
def rotateleft():
    base_rotate_left()
    return ("",204)

@app.route('/turtlebot3/actions/baserotateleftwithtime', methods= ["POST"])
def rotateleftwithtime():
    if request.is_json:
        seconds = request.json
        base_rotate_left_with_time(seconds)
        return ("",204)
    else:
        abort(415)

@app.route('/turtlebot3/actions/baserotateright', methods= ["POST"])
def rotateright():
    base_rotate_right()
    return ("",204)

@app.route('/turtlebot3/actions/baserotaterightwithtime', methods= ["POST"])
def rotaterightwithtime():
    if request.is_json:
        seconds = request.json
        base_rotate_right_with_time(seconds)
        return ("",204)
    else:
        abort(415)

@app.route('/turtlebot3/actions/basestop', methods= ["POST"])
def basestop():
    stop_wheel()
    return ("",204)
# this was a debugg fct so it is not listed in the Thing Description
@app.route('/turtlebot3/actions/arm', methods= ["POST"])
def arm():
    arm_move()
    return ("",204)
# this was a debugg fct so it is not listed in the Thing Description
@app.route('/turtlebot3/actions/shutdown', methods= ["POST"])
def shutdown():
    shutdown_flask()
    return ("",204)

@app.route('/turtlebot3/actions/gripopen', methods= ["POST"])
def gripperopen():
    gripper_open()
    return ("",204)

@app.route('/turtlebot3/actions/gripclose', methods= ["POST"])
def gripperclose():
    gripper_close()
    return ("",204)

@app.route('/turtlebot3/actions/armpose1', methods= ["POST"])
def armpose1():
    arm_front_position1()
    return ("",204)

@app.route('/turtlebot3/actions/armpose2', methods= ["POST"])
def armpose2():
    arm_front_position2()
    return ("",204)

@app.route('/turtlebot3/actions/armpose3', methods= ["POST"])
def armpose3():
    arm_front_position3()
    return ("",204)

@app.route('/turtlebot3/actions/armpose4', methods= ["POST"])
def armpose4():
    arm_left_position1()
    return ("",204)

@app.route('/turtlebot3/actions/armpose5', methods= ["POST"])
def armpose5():
    arm_left_position2()
    return ("",204)

@app.route('/turtlebot3/actions/armpose6', methods= ["POST"])
def armpose6():
    arm_left_position3()
    return ("",204)

@app.route('/turtlebot3/actions/armpose7', methods= ["POST"])
def armpose7():
    arm_right_position1()
    return ("",204)

@app.route('/turtlebot3/actions/armpose8', methods= ["POST"])
def armpose8():
    arm_right_position2()
    return ("",204)

@app.route('/turtlebot3/actions/armpose9', methods= ["POST"])
def armpose9():
    arm_right_position3()
    return ("",204)

def main():

    threading.Thread(rospy.init_node('turtel_control', anonymous=False, log_level = None)).start()
    #Initialize Turtlebot
    init_robot()
    #Start Flask
    threading.Thread(target=app.run(host="172.16.1.250" ,port=8080,use_reloader = False))
    


if __name__ == '__main__':
    main()