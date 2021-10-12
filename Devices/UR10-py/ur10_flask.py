from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_io import RTDEIOInterface as RTDEIO
import requests
import socket
import time
import json
import _thread


from flask import Flask, request, abort,redirect, url_for,flash


from jsonschema import validate
# from jschon import JSON, JSONSchema
# from pprint import pp
from sys import exit


try:
    from PIL import Image
except ImportError:
    exit("This script requires the pillow module\nInstall with: sudo pip install pillow")

# ---------------- CONFIG ----------------

with open("constants.json", "r") as f:
    constants = json.load(f)

with open("convertedTD.json", "r") as f:
    td = json.load(f)

TD_DIRECTORY_ADDRESS = constants["TD_DIRECTORY_ADDRESS"]
LISTENING_PORT = constants["LISTENING_PORT"]
global DEFAULTSPEED
DEFAULTSPEED = constants["DEFAULTSPEED"]
global DEFAULTACCELERATION
DEFAULTACCELERATION = constants["DEFAULTACCELERATION"]
global HOMELOCATION
HOMELOCATION = constants["HOMELOCATION"]


app = Flask(__name__)

rtde_c = RTDEControl("172.16.1.222")
rtde_r = RTDEReceive("172.16.1.222")


@app.route("/ur10/")
def thing_description():
    return json.dumps(td), {'Content-Type': 'application/json'}


@app.route("/ur10/properties/homeloc", methods=["GET"])
def homeloc():
    x = json.dumps(HOMELOCATION)
    print(type(json.dumps(HOMELOCATION)))
    print(json.dumps(HOMELOCATION))
    return x , 200, {'Content-Type': 'application/json'}


@app.route("/ur10/properties/curLocation", methods=["GET"])
def curLocation():
    ##rtde_c = RTDEControl("172.16.1.222")
    ##rtde_r = RTDEReceive("172.16.1.222")
    TCPpose = rtde_r.getActualTCPPose()
    TCPpose[0]= TCPpose[0]*1000
    TCPpose[1]= TCPpose[1]*1000
    TCPpose[2]= (TCPpose[2]-0.4)*1000
    return json.dumps(TCPpose), 200, {'Content-Type': 'application/json'}

@app.route("/ur10/properties/curJointPos", methods=["GET"])
def curJointPos():
    ##rtde_c = RTDEControl("172.16.1.222")
    ##rtde_r = RTDEReceive("172.16.1.222")
    init_q = rtde_r.getActualQ()
    for i in range (6):
        init_q[i]= init_q[i]*57.29
    return json.dumps(init_q), 200, {'Content-Type': 'application/json'}


@app.route("/ur10/properties/moveSpeed", methods=["GET","PUT"])
def speed():
    global DEFAULTSPEED
    if request.method == "PUT":
        if request.is_json:

            print(request.json)
            schema = td["properties"]["moveSpeed"]
            try:
                validate(instance=request.json, schema=schema)
                DEFAULTSPEED = request.json
                return "new default speed is {}".format(DEFAULTSPEED), 200, {'Content-Type': 'application/json'}
            except:
                
                abort(400,"wrong input")


        else:
            abort(415)
    else:
        return json.dumps(DEFAULTSPEED), 200, {'Content-Type': 'application/json'}

@app.route("/ur10/properties/moveAcceleration", methods=["GET","PUT"])
def acceleration():
    global DEFAULTACCELERATION
    if request.method == "PUT":
        if request.is_json:
            print((request.json))
            schema = td["properties"]["moveAcceleration"]
            try:
                validate(instance=request.json, schema=schema)
                DEFAULTACCELERATION = request.json
                return "new default acceleration is {}".format(DEFAULTACCELERATION), 200, {'Content-Type': 'application/json'}
            except:
                
                abort(400, "wrong input")


        else:
            abort(415)
    else:
        return json.dumps(DEFAULTACCELERATION), 200, {'Content-Type': 'application/json'}


@app.route("/ur10/actions/goHome", methods=["POST"])
def goHome():
    ##rtde_c = RTDEControl("172.16.1.222")
    ##rtde_r = RTDEReceive("172.16.1.222")
    list1 = []
    global HOMELOCATION
    for key, value in  HOMELOCATION.items():
        list1.append(value)
    print(list1)
    jointPoslist = []
    for i in range (6):
        jointPoslist.append(list1[i+3])
    print(jointPoslist)
    for i in range (6):
        jointPoslist[i]= jointPoslist[i]/57.29 ## is the ratio between the angle of joint in degrees on the control screen and the angle output when API function is used. (https://sdurobotics.gitlab.io/ur_rtde/api/api.html#rtde-control-interface-api)
    status = rtde_r.getRobotStatus()
    if status == 3:
        rtde_c.moveJ(jointPoslist, DEFAULTSPEED, DEFAULTACCELERATION, False)
        return "" ,204
    else:
    
        abort(400,"robot is not in Normal mode")

@app.route("/ur10/actions/turnBase", methods=["POST"])
def turnBase():

    if request.is_json:

        schema = td["actions"]["turnBase"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
         
            abort(400,"wrong input")

        degree = request.json["base"]
        ##rtde_c = RTDEControl("172.16.1.222")
        ##rtde_r = RTDEReceive("172.16.1.222")
        init_q = rtde_r.getActualQ()
        new_q = init_q[:]
        new_q[0] += degree/57.29
        status = rtde_r.getRobotStatus()
        if status == 3:
            rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, False)
            return "", 204 
        else:
       
            abort(400,"robot is not in Normal mode")
    else:
        
        abort(415,"Error 415")


##################################################3
@app.route("/ur10/actions/turnShoulder", methods=["POST"])
def turnShoulder():

    if request.is_json:
        # schema = td["actions"]["turnBase"]["input"]
        # valid_input = Draft6Validator(schema).is_valid(request.json)
 
        schema = td["actions"]["turnShoulder"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
            
            abort(400,"wrong input")

        
        degree = request.json["shoulder"]
        ##rtde_c = RTDEControl("172.16.1.222")
        ##rtde_r = RTDEReceive("172.16.1.222")
        init_q = rtde_r.getActualQ()
        new_q = init_q[:]
        new_q[1] += degree/57.29
        status = rtde_r.getRobotStatus()
        if status == 3:
            rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, False)
            return "", 204 
        else:
            return "robot is not in Normal mode"
            abort(400)

    else:
        return "Error 415"
        abort(415)


@app.route("/ur10/actions/turnElbow", methods=["POST"])
def turnElbow():

    if request.is_json:
        # schema = td["actions"]["turnBase"]["input"]
        # valid_input = Draft6Validator(schema).is_valid(request.json)

        schema = td["actions"]["turnElbow"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
            
            abort(400,"wrong input")



        degree = request.json["elbow"]
        print((type(degree)))
        ##rtde_c = RTDEControl("172.16.1.222")
        ##rtde_r = RTDEReceive("172.16.1.222")
        init_q = rtde_r.getActualQ()
        new_q = init_q[:]
        new_q[2] += degree/57.29
        status = rtde_r.getRobotStatus()
        if status == 3:
            rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, False)
            return "", 204 
        else:
            
            abort(400,"robot is not in Normal mode")
    else:
        
        abort(415,"Error 415")


@app.route("/ur10/actions/turnWrist1", methods=["POST"])
def turnWrist1():

    if request.is_json:
        # schema = td["actions"]["turnBase"]["input"]
        # valid_input = Draft6Validator(schema).is_valid(request.json)
        
        schema = td["actions"]["turnWrist1"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
          
            abort(400, "wrong input")


        degree = request.json["wrist1"]
        print((type(degree)))
        ##rtde_c = RTDEControl("172.16.1.222")
        ##rtde_r = RTDEReceive("172.16.1.222")
        init_q = rtde_r.getActualQ()
        new_q = init_q[:]
        new_q[3] += degree/57.29
        status = rtde_r.getRobotStatus()
        if status == 3:
            rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, False)
            return "", 204 
        else:
            abort(400, "robot is not in Normal mode")
        
    else:
       
        abort(415, "Error 415")


@app.route("/ur10/actions/turnWrist2", methods=["POST"])
def turnWrist2():

    if request.is_json:
        # schema = td["actions"]["turnBase"]["input"]
        # valid_input = Draft6Validator(schema).is_valid(request.json)
        
        schema = td["actions"]["turnWrist2"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
            
            abort(400,"wrong input")


        degree = request.json["wrist2"]
        print((type(degree)))
        ##rtde_c = RTDEControl("172.16.1.222")
        ##rtde_r = RTDEReceive("172.16.1.222")
        init_q = rtde_r.getActualQ()
        new_q = init_q[:]
        new_q[4] += degree/57.29
        status = rtde_r.getRobotStatus()
        if status == 3:
            rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, False)
            return "", 204 
        else:
           
            abort(400,"robot is not in Normal mode")

    else:
        
        abort(415,"Error 415")


@app.route("/ur10/actions/turnWrist3", methods=["POST"])
def turnWrist3():

    if request.is_json:
        #schema = td["actions"]["turnWrist3"]["input"]
        # valid_input = Draft6Validator(schema).is_valid(request.json)
        print(request.json)
        schema = td["actions"]["turnWrist3"]["input"]
        try:
            v = validate(instance=request.json, schema=schema)
            print(v)
            if jsonschema.exceptions.ValidationError:
                return "got in heeeere"
        except:
            
            abort(400,"wrong input")


        degree = request.json["wrist3"]
        print((type(degree)))
        ##rtde_c = RTDEControl("172.16.1.222")
        ##rtde_r = RTDEReceive("172.16.1.222")
        init_q = rtde_r.getActualQ()
        new_q = init_q[:]
        new_q[5] += degree/57.29
        status = rtde_r.getRobotStatus()
        if status == 3:
            rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, False)
            return "", 204 
        else:
            abort(400,"wrong input")

    else:
        
        abort(415,"Error 415")     

#and type(value)== float and -360<value<360
@app.route("/ur10/actions/setJointDegrees", methods=["POST"])
def setJointDegrees():
    print(request.json)
    if request.is_json:
        print(request.json)
        jointList = [None]*6
        print(request.json)
        asyn = False

        schema = td["actions"]["setJointDegrees"]["input"]
        try:
            validate(instance=request.json, schema=schema)
            for key in request.json.keys():
                if key == "base" :
                    jointList[0] = request.json["base"]/57.29
                elif key == "shoulder":
                    jointList[1] = request.json["shoulder"]/57.29
                elif key == "elbow":
                    jointList[2] = request.json["elbow"]/57.29
                elif key == "wrist1":
                    jointList[3] = request.json["wrist1"]/57.29
                elif key == "wrist2":
                    jointList[4] = request.json["wrist2"]/57.29
                elif key == "wrist3":
                    jointList[5] = request.json["wrist3"]/57.29
                elif key == "async":
                    asyn = request.json["async"]

            print(jointList)
            ##rtde_c = RTDEControl("172.16.1.222")
            ##rtde_r = RTDEReceive("172.16.1.222")
            init_q = rtde_r.getActualQ()
            print(init_q)

            for i in range (6):
                if not jointList[i] == None:
                    init_q[i]=jointList[i]
                else:
                    pass
            status = rtde_r.getRobotStatus()      
            if status == 3:            
                new_q = init_q[:]
                print(new_q)
                rtde_c.moveJ(new_q, DEFAULTSPEED, DEFAULTACCELERATION, asyn)
                return "", 204 
            else:
                
                abort(400,"robot is not in Normal mode")
        except Exception as e:
            print(e)
            
            abort(400,"wrong input")


        
    else:
        
        abort(415,"Error 415")  

###################################################333

@app.route("/ur10/actions/goTo", methods=["POST"])
def goTo():
    global DEFAULTACCELERATION
    global DEFAULTSPEED
    if request.is_json:
        goList = [None]*6
        print(request.json)
        print(type(request.json))

        schema = td["actions"]["goTo"]["input"]
        try:
            validate(instance=request.json, schema=schema)
        except:
            
            abort(400,"wrong input")
        asyn = False

        for key in request.json.keys():
            if key == "x" :
                goList[0] = request.json["x"]/1000 
            elif key == "y":
                goList[1] = request.json["y"]/1000 
            elif key == "z":
                goList[2] = request.json["z"]/1000 + 0.4
            elif key == "rx" :
                goList[3] = request.json["rx"] 
            elif key == "ry":
                goList[4] = request.json["ry"]
            elif key == "rz":
                goList[5] = request.json["rz"]
            elif key == "s" and 0 < request.json["s"] <= 1:
                DEFAULTSPEED = request.json["s"]
            elif key == "a"and 0 < request.json["a"] <= 1:
                DEFAULTACCELERATION = request.json["a"]
            elif key == "async":
                asyn = request.json["async"]


        print(goList)
        ##rtde_c = RTDEControl("172.16.1.222")
        ##rtde_r = RTDEReceive("172.16.1.222")
        TCPpose = rtde_r.getActualTCPPose()
        print(TCPpose)

        for i in range (6):
            if not goList[i] == None:
                TCPpose[i]=goList[i]
            else:
                pass  

        new_q = TCPpose[:]
        print(new_q)
        status = rtde_r.getRobotStatus()
        if status == 3:
            rtde_c.moveJ_IK(new_q,DEFAULTSPEED,DEFAULTACCELERATION , asyn)
            TCPpose = rtde_r.getActualTCPPose()
            TCPpose[0]= TCPpose[0]*1000
            TCPpose[1]= TCPpose[1]*1000
            TCPpose[2]= (TCPpose[2]-0.4)*1000
            TCPpose[3]= TCPpose[3]
            TCPpose[4]= TCPpose[4]
            TCPpose[5]= TCPpose[5]
            return "", 204
        else:
           
            abort(400,"robot is not in Normal mode") 
    else:
        
        abort(415,"Error 415")  


@app.route("/ur10/actions/gripClose", methods=["POST"])
def gripClose(): 

    rtde_io_ = RTDEIO("172.16.1.222")
    rtde_receive_ = RTDEReceive("172.16.1.222")
    rtde_io_.setStandardDigitalOut(0, False)
    rtde_io_.setStandardDigitalOut(1, True)
    rtde_io_.setStandardDigitalOut(3, False)
    return "", 204

@app.route("/ur10/actions/gripCloseLight", methods=["POST"])
def gripCloseLight(): 

    rtde_io_ = RTDEIO("172.16.1.222")
    rtde_receive_ = RTDEReceive("172.16.1.222")
    rtde_io_.setStandardDigitalOut(0, False)
    rtde_io_.setStandardDigitalOut(1, False)
    rtde_io_.setStandardDigitalOut(3, True)
    return "", 204

@app.route("/ur10/actions/gripOpen", methods=["POST"])
def gripOpen(): 

    rtde_io_ = RTDEIO("172.16.1.222")
    rtde_receive_ = RTDEReceive("172.16.1.222")
    rtde_io_.setStandardDigitalOut(0, True)
    rtde_io_.setStandardDigitalOut(1, False)
    rtde_io_.setStandardDigitalOut(3, False)
    return "", 204

##################################33

def submit_td(ip_addr, tdd_address):
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
                #print("TD could not be uploaded. Will try again in 15 Seconds...")
                time.sleep(45)
        except Exception as e:
            #print(e)
            #print("TD could not be uploaded. Will try again in 15 Seconds...")
            time.sleep(45)


# wait for Wifi to connect
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
while True:
    try:
        # connect to router to ensure a successful connection
        s.connect((constants["routerIP"],constants["routerPort"]))
        ip_addr = s.getsockname()[0] + ":" + str(LISTENING_PORT)
        print(ip_addr)
        break
    except OSError:
        time.sleep(5)

# Submit TD to directory
_thread.start_new_thread(submit_td, (ip_addr, TD_DIRECTORY_ADDRESS))

# Run app server
app.run(host='0.0.0.0', port=8080)