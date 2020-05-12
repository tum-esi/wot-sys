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

class angle4th {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle4th = null;
    }
    else {
      if (initObj.hasOwnProperty('angle4th')) {
        this.angle4th = initObj.angle4th
      }
      else {
        this.angle4th = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type angle4th
    // Serialize message field [angle4th]
    bufferOffset = _serializer.float64(obj.angle4th, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type angle4th
    let len;
    let data = new angle4th(null);
    // Deserialize message field [angle4th]
    data.angle4th = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'swiftpro/angle4th';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8eecd591854543ff5e9cf583de2d05e6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 angle4th
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new angle4th(null);
    if (msg.angle4th !== undefined) {
      resolved.angle4th = msg.angle4th;
    }
    else {
      resolved.angle4th = 0.0
    }

    return resolved;
    }
};

module.exports = angle4th;
