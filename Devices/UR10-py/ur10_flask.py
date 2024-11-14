from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_io import RTDEIOInterface as RTDEIO

import requests
import socket
import time
import json
import _thread
import argparse  # Import argparse for command-line arguments

from flask import Flask, request, abort, redirect, url_for, flash
from jsonschema import validate
from sys import exit
import time
import math

# Parse command-line arguments
parser = argparse.ArgumentParser(description="UR10 Flask Server")
parser.add_argument('--dummy', action='store_true',
                    help="Run in dummy mode without connecting to the robot")
args = parser.parse_args()

# ---------------- CONFIG ----------------
with open("constants.json", "r") as f:
    constants = json.load(f)

with open("convertedTD.json", "r") as f:
    td = json.load(f)

with open("config.json", "r") as f:
    config = json.load(f)

TD_DIRECTORY_ADDRESS = constants["TD_DIRECTORY_ADDRESS"]
LISTENING_PORT = constants["LISTENING_PORT"]
global DEFAULTSPEED
DEFAULTSPEED = constants["DEFAULTSPEED"]
global DEFAULTACCELERATION
DEFAULTACCELERATION = constants["DEFAULTACCELERATION"]
global HOMELOCATION
HOMELOCATION = constants["HOMELOCATION"]

app = Flask(__name__)

# Initialize robot interfaces only if not in dummy mode

rtde_c = None
rtde_r = None
rtde_io_ = None


def initialize_robot_interfaces():
    global rtde_c
    global rtde_r
    global rtde_io_
    rtde_c = RTDEControl(config["robotIP"], RTDEControl.FLAG_CUSTOM_SCRIPT | RTDEControl.FLAG_NO_WAIT)
    rtde_r = RTDEReceive(config["robotIP"])
    rtde_io_ = RTDEIO(config["robotIP"])


def close_robot_interfaces():
    global rtde_c
    global rtde_r
    global rtde_io_
    print("Close Robot triggered")
    try:
        if rtde_c is not None:
            rtde_c.stopScript()  # Stop any running script on the controller
            rtde_c.disconnect()  # Disconnect the RTDE Control interface
            rtde_c = None  # Clear the reference

        if rtde_r is not None:
            rtde_r.disconnect()  # Disconnect the RTDE Receive interface
            rtde_r = None  # Clear the reference

        # No disconnect for rtde_io_ since it doesn't have a disconnect method
        rtde_io_ = None  # Just clear the reference

        print("Robot interfaces successfully closed.")
    except Exception as e:
        print(f"Error while closing robot interfaces: {e}")



def reinitialize_robot_interfaces():
    close_robot_interfaces()  # Close existing interfaces
    time.sleep(1)  # Short delay to ensure resources are released
    initialize_robot_interfaces()  # Reinitialize the interfaces

if not args.dummy:
    initialize_robot_interfaces()
else:
    rtde_c = None
    rtde_r = None
    rtde_io_ = None


@app.route("/ur10/")
def thing_description():
    return json.dumps(td), {"Content-Type": "application/json"}

@ app.route("/ur10/properties/getSafetyStatus")
def getSafetyStatus():
    statusBits = rtde_r.getSafetyStatusBits()
    return json.dumps(statusBits), 200, {"Content-Type": "application/json"}


@app.route("/ur10/properties/jointConfiguration", methods=["GET"])
def jointConfiguration():
    return json.dumps({"jointConfiguration": config["jointConfiguration"]}), 200, {"Content-Type": "application/json"}

@app.route("/ur10/actions/closeGripper", methods=["POST"])
def grip_close():
    print ("closing gripper...")    
    result = rtde_io_.setInputIntRegister(18, 1)
    print(result)
    if result:
        return "", 204
    else:
        abort(400, "Couldn't close gripper")
    

@app.route("/ur10/actions/openGripper", methods=["POST"])
def grip_open():
    print ("opening gripper...")
    result = rtde_io_.setInputIntRegister(18, 2)
    print(result)
    if result:
        return "", 204
    else:
        abort(400, "Couldn't open gripper")


def apply_offset(coordinates, offset):
    # Apply position offset
    x = coordinates[0] + offset['position']['x']
    y = coordinates[1] + offset['position']['y']
    z = coordinates[2] + offset['position']['z']

    # Apply rotation offset (this is a simplified rotation - for accurate rotation, consider using rotation matrices or quaternions)
    rx = coordinates[3] + math.radians(offset['rotation']['rx'])
    ry = coordinates[4] + math.radians(offset['rotation']['ry'])
    rz = coordinates[5] + math.radians(offset['rotation']['rz'])

    return [x, y, z, rx, ry, rz]


@ app.route("/ur10/properties/worldCoordinates", methods=["GET"])
def worldCoordinates():
    if args.dummy:
        return json.dumps([0, 0, 0, 0, 0, 0]), 200, {"Content-Type": "application/json"}
    TCPpose = rtde_r.getActualTCPPose()
    TCPpose[0] = TCPpose[0] * 1000
    TCPpose[1] = TCPpose[1] * 1000
    TCPpose[2] = (TCPpose[2] - 0.4) * 1000
    world_pose = apply_offset(TCPpose, config["offset"])
    return json.dumps(world_pose), 200, {"Content-Type": "application/json"}


@ app.route("/ur10/properties/offset", methods=["GET"])
def get_offset():
    return json.dumps(config["offset"]), 200, {"Content-Type": "application/json"}


@ app.route("/ur10/properties/homePosition", methods=["GET"])
def homePosition():
    x = json.dumps(HOMELOCATION)
    return x, 200, {"Content-Type": "application/json"}


@ app.route("/ur10/properties/currentCoordinates", methods=["GET"])
def currentCoordinates():
    if args.dummy:
        return json.dumps([0, 0, 0, 0, 0, 0]), 200, {"Content-Type": "application/json"}
    TCPpose = rtde_r.getActualTCPPose()
    TCPpose[0] = TCPpose[0] * 1000
    TCPpose[1] = TCPpose[1] * 1000
    TCPpose[2] = (TCPpose[2] - 0.4) * 1000
    return json.dumps(TCPpose), 200, {"Content-Type": "application/json"}


@ app.route("/ur10/properties/currentJointDegrees", methods=["GET"])
def currentJointDegrees():
    if args.dummy:
        return json.dumps([0, 0, 0, 0, 0, 0]), 200, {"Content-Type": "application/json"}
    init_q = rtde_r.getActualQ()
    for i in range(6):
        init_q[i] = init_q[i] * 57.29
    return json.dumps(init_q), 200, {"Content-Type": "application/json"}


@ app.route("/ur10/properties/moveSpeed", methods=["GET", "PUT"])
def speed():
    global DEFAULTSPEED
    if request.method == "PUT":
        if request.is_json:
            print(request.json)
            schema = td["properties"]["moveSpeed"]
            try:
                validate(instance=request.json, schema=schema)
                DEFAULTSPEED = request.json
                return 200, {"Content-Type": "application/json"}
            except:
                abort(400, "wrong input")
        else:
            abort(415)
    else:
        return json.dumps(DEFAULTSPEED), 200, {"Content-Type": "application/json"}


@ app.route("/ur10/properties/moveAcceleration", methods=["GET", "PUT"])
def acceleration():
    global DEFAULTACCELERATION
    if request.method == "PUT":
        if request.is_json:
            print((request.json))
            schema = td["properties"]["moveAcceleration"]
            try:
                validate(instance=request.json, schema=schema)
                DEFAULTACCELERATION = request.json
                return 200, {"Content-Type": "application/json"}
            except:
                abort(400, "wrong input")
        else:
            abort(415)
    else:
        return (
            json.dumps(DEFAULTACCELERATION),
            200,
            {"Content-Type": "application/json"},
        )


@ app.route("/ur10/properties/urdfFile", methods=["GET"])
def urdfFile():
    urdf_url = "http://127.0.0.1:5000/files/ur10_urdf.zip"
    return json.dumps({"url": urdf_url}), 200, {"Content-Type": "application/json"}


@ app.route("/ur10/actions/goHome", methods=["POST"])
def goHome():
    if args.dummy:
        return "", 204
    list1 = []
    global HOMELOCATION
    for key, value in HOMELOCATION.items():
        list1.append(value)
    print(list1)
    jointPoslist = []
    for i in range(6):
        jointPoslist.append(list1[i + 3])
    print(jointPoslist)
    for i in range(6):
        jointPoslist[i] = (jointPoslist[i] / 57.29)
    status = rtde_r.getSafetyStatusBits()
    if status >= 1:
        rtde_c.moveJ(jointPoslist, DEFAULTSPEED, DEFAULTACCELERATION, False)
        return "", 204
    else:
        abort(400, "robot is not in Normal mode")


@ app.route("/ur10/actions/turnBase", methods=["POST"])
def turnBase():
    if args.dummy:
        return "", 204
    if request.is_json:
        schema = td["actions"]["turnBase"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
            abort(400, "wrong input")
        degree = request.json["base"]
        init_q = rtde_r.getActualQ()
        new_q = init_q[:]
        new_q[0] += degree / 57.29
        status = rtde_r.getSafetyStatusBits()
        if status >= 1:
            isDone = rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, False)
            if isDone:
                return "", 204
            else:
                abort(400, "Couldn't perform")
        else:
            abort(400, "robot is not in Normal mode")
    else:
        abort(415, "Error 415")


@ app.route("/ur10/actions/turnShoulder", methods=["POST"])
def turnShoulder():
    if args.dummy:
        return "", 204
    if request.is_json:
        schema = td["actions"]["turnShoulder"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
            abort(400, "wrong input")
        degree = request.json["shoulder"]
        init_q = rtde_r.getActualQ()
        new_q = init_q[:]
        new_q[1] += degree / 57.29
        status = rtde_r.getSafetyStatusBits()
        print(status)
        if status >= 1:
            isDone = rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, False)
            if isDone:
                return "", 204
            else:
                abort(400, "Couldn't perform")
        else:
            return "robot is not in Normal mode"
            abort(400)
    else:
        return "Error 415"
        abort(415)


@ app.route("/ur10/actions/turnElbow", methods=["POST"])
def turnElbow():
    if args.dummy:
        return "", 204
    if request.is_json:
        schema = td["actions"]["turnElbow"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
            abort(400, "wrong input")
        degree = request.json["elbow"]
        print((type(degree)))
        init_q = rtde_r.getActualQ()
        new_q = init_q[:]
        new_q[2] += degree / 57.29
        status = rtde_r.getSafetyStatusBits()
        if status >= 1:
            isDone = rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, False)
            if isDone:
                return "", 204
            else:
                abort(400, "Couldn't perform")
        else:
            abort(400, "robot is not in Normal mode")
    else:
        abort(415, "Error 415")


@ app.route("/ur10/actions/turnWrist1", methods=["POST"])
def turnWrist1():
    if args.dummy:
        return "", 204
    if request.is_json:
        schema = td["actions"]["turnWrist1"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
            abort(400, "wrong input")
        degree = request.json["wrist1"]
        print((type(degree)))
        init_q = rtde_r.getActualQ()
        new_q = init_q[:]
        new_q[3] += degree / 57.29
        status = rtde_r.getSafetyStatusBits()
        if status >= 1:
            isDone = rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, False)
            if isDone:
                return "", 204
            else:
                abort(400, "Couldn't perform")
        else:
            abort(400, "robot is not in Normal mode")
    else:
        abort(415, "Error 415")


@ app.route("/ur10/actions/turnWrist2", methods=["POST"])
def turnWrist2():
    if args.dummy:
        return "", 204
    if request.is_json:
        schema = td["actions"]["turnWrist2"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
            abort(400, "wrong input")
        degree = request.json["wrist2"]
        print((type(degree)))
        init_q = rtde_r.getActualQ()
        new_q = init_q[:]
        new_q[4] += degree / 57.29
        status = rtde_r.getSafetyStatusBits()
        if status >= 1:
            isDone = rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, False)
            if isDone:
                return "", 204
            else:
                abort(400, "Couldn't perform")
        else:
            abort(400, "robot is not in Normal mode")
    else:
        abort(415, "Error 415")


@ app.route("/ur10/actions/turnWrist3", methods=["POST"])
def turnWrist3():
    if args.dummy:
        return "", 204
    if request.is_json:
        print(request.json)
        schema = td["actions"]["turnWrist3"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
            abort(400, "wrong input")
        degree = request.json["wrist3"]
        print((type(degree)))
        init_q = rtde_r.getActualQ()
        new_q = init_q[:]
        new_q[5] += degree / 57.29
        status = rtde_r.getSafetyStatusBits()
        if status >= 1:
            isDone = rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, False)
            if isDone:
                return "", 204
            else:
                abort(400, "Couldn't perform")
        else:
            abort(400, "wrong input")
    else:
        abort(415, "Error 415")


@ app.route("/ur10/actions/setJointDegrees", methods=["POST"])
def setJointDegrees():
    if args.dummy:
        return "", 204
    print(request.json)

    global rtde_c
    if request.is_json:
        print(request.json)
        jointList = [None] * 6
        print(request.json)
        asyn = False
        schema = td["actions"]["setJointDegrees"]["input"]
        try:
            validate(instance=request.json, schema=schema)
            if (type(request.json) == dict):
                for key in request.json.keys():
                    if key == "base":
                        jointList[0] = request.json["base"] / 57.29
                    elif key == "shoulder":
                        jointList[1] = request.json["shoulder"] / 57.29
                    elif key == "elbow":
                        jointList[2] = request.json["elbow"] / 57.29
                    elif key == "wrist1":
                        jointList[3] = request.json["wrist1"] / 57.29
                    elif key == "wrist2":
                        jointList[4] = request.json["wrist2"] / 57.29
                    elif key == "wrist3":
                        jointList[5] = request.json["wrist3"] / 57.29
                    elif key == "async":
                        asyn = request.json["async"]
            elif (type(request.json) == list):
                for i in range(6):
                    jointList[i] = request.json[i] / \
                        57.29  # convert to radians
            print("jointList:")
            print(jointList)

            init_q = rtde_r.getActualQ()
            print("init_q:")
            print(init_q)
            for i in range(6):
                if not jointList[i] == None:
                    init_q[i] = jointList[i]
                else:
                    pass
            status = rtde_r.getSafetyStatusBits()
            if status >= 1:
                new_q = init_q[:]
                print("new_q:")
                print(new_q)

                isDone = rtde_c.moveJ(
                    new_q, DEFAULTSPEED, DEFAULTACCELERATION, asyn)
                if not isDone:
                    # try with Reinitializing
                    reinitialize_robot_interfaces()
                    isDone = rtde_c.moveJ(
                        new_q, DEFAULTSPEED, DEFAULTACCELERATION, asyn)

                if isDone:
                    return "", 204
                else:
                    abort(400, "Couldn't perform after retry")
            else:
                print("robot is not in Normal mode")
                print(rtde_r.getSafetyStatusBits())
                abort(400, "robot is not in Normal mode")
        except Exception as e:
            print(e)
            abort(400, "wrong input")
    else:
        abort(415, "Error 415")


@ app.route("/ur10/actions/goTo", methods=["POST"])
def goTo():
    if args.dummy:
        return "", 204
    global DEFAULTACCELERATION
    global DEFAULTSPEED
    if request.is_json:
        goList = [None] * 6
        print(request.json)
        print(type(request.json))
        schema = td["actions"]["goTo"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
            abort(400, "wrong input")
        asyn = False
        for key in request.json.keys():
            if key == "x":
                goList[0] = request.json["x"] / 1000
            elif key == "y":
                goList[1] = request.json["y"] / 1000
            elif key == "z":
                goList[2] = request.json["z"] / 1000 + 0.4
            elif key == "rx":
                goList[3] = request.json["rx"]
            elif key == "ry":
                goList[4] = request.json["ry"]
            elif key == "rz":
                goList[5] = request.json["rz"]
            elif key == "s" and 0 < request.json["s"] <= 1:
                DEFAULTSPEED = request.json["s"]
            elif key == "a" and 0 < request.json["a"] <= 1:
                DEFAULTACCELERATION = request.json["a"]
            elif key == "async":
                asyn = request.json["async"]
        print(goList)
        TCPpose = rtde_r.getActualTCPPose()
        print(TCPpose)
        for i in range(6):
            if not goList[i] == None:
                TCPpose[i] = goList[i]
            else:
                pass
        new_q = TCPpose[:]
        print(new_q)
        status = rtde_r.getSafetyStatusBits()
        if status >= 1:
            isDone = rtde_c.moveJ_IK(new_q, DEFAULTSPEED, DEFAULTACCELERATION, asyn)
            if isDone:
                TCPpose = rtde_r.getActualTCPPose()
                TCPpose[0] = TCPpose[0] * 1000
                TCPpose[1] = TCPpose[1] * 1000
                TCPpose[2] = (TCPpose[2] - 0.4) * 1000
                TCPpose[3] = TCPpose[3]
                TCPpose[4] = TCPpose[4]
                TCPpose[5] = TCPpose[5]
                return "", 204
            else:
                abort(400, "Couldn't perform")
            return "", 204
        else:
            abort(400, "robot is not in Normal mode")
    else:
        abort(415, "Error 415")


def submit_td(tdd_address):
    global td
    td = td
    print("Uploading TD to directory ...")
    while True:
        try:
            r = requests.post("{}/td".format(tdd_address), json=td)
            r.close()
            print("Got response: ", r.status_code)
            if 200 <= r.status_code <= 299:
                print("TD uploaded!")
                return
            else:
                time.sleep(45)
        except Exception as e:
            time.sleep(45)


if not args.dummy:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    while True:
        try:
            break
        except OSError:
            time.sleep(5)

app.run(host="0.0.0.0", port=8080)
