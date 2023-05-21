// Auto-generated. Do not edit!

// (in-package parc_robot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Weed = require('./Weed.js');

//-----------------------------------------------------------

class WeedDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.weeds = null;
    }
    else {
      if (initObj.hasOwnProperty('weeds')) {
        this.weeds = initObj.weeds
      }
      else {
        this.weeds = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WeedDetection
    // Serialize message field [weeds]
    // Serialize the length for message field [weeds]
    bufferOffset = _serializer.uint32(obj.weeds.length, buffer, bufferOffset);
    obj.weeds.forEach((val) => {
      bufferOffset = Weed.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WeedDetection
    let len;
    let data = new WeedDetection(null);
    // Deserialize message field [weeds]
    // Deserialize array length for message field [weeds]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.weeds = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.weeds[i] = Weed.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 16 * object.weeds.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'parc_robot/WeedDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '845a4e3a48d0573bc750910e15745135';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # A list of detected weeds
    Weed[] weeds
    
    
    ================================================================================
    MSG: parc_robot/Weed
    # A weed in the field
    float64 latitude
    float64 longitude
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WeedDetection(null);
    if (msg.weeds !== undefined) {
      resolved.weeds = new Array(msg.weeds.length);
      for (let i = 0; i < resolved.weeds.length; ++i) {
        resolved.weeds[i] = Weed.Resolve(msg.weeds[i]);
      }
    }
    else {
      resolved.weeds = []
    }

    return resolved;
    }
};

module.exports = WeedDetection;
