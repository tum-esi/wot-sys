// Auto-generated. Do not edit!

// (in-package swiftpro.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SwiftproState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.motor_angle1 = null;
      this.motor_angle2 = null;
      this.motor_angle3 = null;
      this.motor_angle4 = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.pump = null;
      this.swiftpro_status = null;
      this.gripper = null;
    }
    else {
      if (initObj.hasOwnProperty('motor_angle1')) {
        this.motor_angle1 = initObj.motor_angle1
      }
      else {
        this.motor_angle1 = 0.0;
      }
      if (initObj.hasOwnProperty('motor_angle2')) {
        this.motor_angle2 = initObj.motor_angle2
      }
      else {
        this.motor_angle2 = 0.0;
      }
      if (initObj.hasOwnProperty('motor_angle3')) {
        this.motor_angle3 = initObj.motor_angle3
      }
      else {
        this.motor_angle3 = 0.0;
      }
      if (initObj.hasOwnProperty('motor_angle4')) {
        this.motor_angle4 = initObj.motor_angle4
      }
      else {
        this.motor_angle4 = 0.0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('pump')) {
        this.pump = initObj.pump
      }
      else {
        this.pump = 0;
      }
      if (initObj.hasOwnProperty('swiftpro_status')) {
        this.swiftpro_status = initObj.swiftpro_status
      }
      else {
        this.swiftpro_status = 0;
      }
      if (initObj.hasOwnProperty('gripper')) {
        this.gripper = initObj.gripper
      }
      else {
        this.gripper = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SwiftproState
    // Serialize message field [motor_angle1]
    bufferOffset = _serializer.float64(obj.motor_angle1, buffer, bufferOffset);
    // Serialize message field [motor_angle2]
    bufferOffset = _serializer.float64(obj.motor_angle2, buffer, bufferOffset);
    // Serialize message field [motor_angle3]
    bufferOffset = _serializer.float64(obj.motor_angle3, buffer, bufferOffset);
    // Serialize message field [motor_angle4]
    bufferOffset = _serializer.float64(obj.motor_angle4, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [pump]
    bufferOffset = _serializer.uint8(obj.pump, buffer, bufferOffset);
    // Serialize message field [swiftpro_status]
    bufferOffset = _serializer.uint8(obj.swiftpro_status, buffer, bufferOffset);
    // Serialize message field [gripper]
    bufferOffset = _serializer.uint8(obj.gripper, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SwiftproState
    let len;
    let data = new SwiftproState(null);
    // Deserialize message field [motor_angle1]
    data.motor_angle1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [motor_angle2]
    data.motor_angle2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [motor_angle3]
    data.motor_angle3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [motor_angle4]
    data.motor_angle4 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pump]
    data.pump = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [swiftpro_status]
    data.swiftpro_status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gripper]
    data.gripper = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 59;
  }

  static datatype() {
    // Returns string type for a message object
    return 'swiftpro/SwiftproState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bcd9671f860a15ba5765d673098d21bb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 motor_angle1
    float64 motor_angle2
    float64 motor_angle3
    float64 motor_angle4
    float64 x
    float64 y
    float64 z
    uint8 	pump
    uint8 	swiftpro_status
    uint8 	gripper
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SwiftproState(null);
    if (msg.motor_angle1 !== undefined) {
      resolved.motor_angle1 = msg.motor_angle1;
    }
    else {
      resolved.motor_angle1 = 0.0
    }

    if (msg.motor_angle2 !== undefined) {
      resolved.motor_angle2 = msg.motor_angle2;
    }
    else {
      resolved.motor_angle2 = 0.0
    }

    if (msg.motor_angle3 !== undefined) {
      resolved.motor_angle3 = msg.motor_angle3;
    }
    else {
      resolved.motor_angle3 = 0.0
    }

    if (msg.motor_angle4 !== undefined) {
      resolved.motor_angle4 = msg.motor_angle4;
    }
    else {
      resolved.motor_angle4 = 0.0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.pump !== undefined) {
      resolved.pump = msg.pump;
    }
    else {
      resolved.pump = 0
    }

    if (msg.swiftpro_status !== undefined) {
      resolved.swiftpro_status = msg.swiftpro_status;
    }
    else {
      resolved.swiftpro_status = 0
    }

    if (msg.gripper !== undefined) {
      resolved.gripper = msg.gripper;
    }
    else {
      resolved.gripper = 0
    }

    return resolved;
    }
};

module.exports = SwiftproState;
