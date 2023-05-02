// Auto-generated. Do not edit!

// (in-package cv_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class grasp_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.grasp_x_cm = null;
      this.grasp_y_cm = null;
      this.grasp_theta = null;
    }
    else {
      if (initObj.hasOwnProperty('grasp_x_cm')) {
        this.grasp_x_cm = initObj.grasp_x_cm
      }
      else {
        this.grasp_x_cm = 0.0;
      }
      if (initObj.hasOwnProperty('grasp_y_cm')) {
        this.grasp_y_cm = initObj.grasp_y_cm
      }
      else {
        this.grasp_y_cm = 0.0;
      }
      if (initObj.hasOwnProperty('grasp_theta')) {
        this.grasp_theta = initObj.grasp_theta
      }
      else {
        this.grasp_theta = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type grasp_msg
    // Serialize message field [grasp_x_cm]
    bufferOffset = _serializer.float32(obj.grasp_x_cm, buffer, bufferOffset);
    // Serialize message field [grasp_y_cm]
    bufferOffset = _serializer.float32(obj.grasp_y_cm, buffer, bufferOffset);
    // Serialize message field [grasp_theta]
    bufferOffset = _serializer.float32(obj.grasp_theta, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type grasp_msg
    let len;
    let data = new grasp_msg(null);
    // Deserialize message field [grasp_x_cm]
    data.grasp_x_cm = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [grasp_y_cm]
    data.grasp_y_cm = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [grasp_theta]
    data.grasp_theta = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cv_pkg/grasp_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d3c4f7d62e5e5364236dabf83a36004';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 grasp_x_cm
    float32 grasp_y_cm
    float32 grasp_theta
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new grasp_msg(null);
    if (msg.grasp_x_cm !== undefined) {
      resolved.grasp_x_cm = msg.grasp_x_cm;
    }
    else {
      resolved.grasp_x_cm = 0.0
    }

    if (msg.grasp_y_cm !== undefined) {
      resolved.grasp_y_cm = msg.grasp_y_cm;
    }
    else {
      resolved.grasp_y_cm = 0.0
    }

    if (msg.grasp_theta !== undefined) {
      resolved.grasp_theta = msg.grasp_theta;
    }
    else {
      resolved.grasp_theta = 0.0
    }

    return resolved;
    }
};

module.exports = grasp_msg;
