// Auto-generated. Do not edit!

// (in-package diffusion_policy.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class get_velocity_list {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.velocity_list = null;
    }
    else {
      if (initObj.hasOwnProperty('velocity_list')) {
        this.velocity_list = initObj.velocity_list
      }
      else {
        this.velocity_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type get_velocity_list
    // Serialize message field [velocity_list]
    bufferOffset = _arraySerializer.float32(obj.velocity_list, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type get_velocity_list
    let len;
    let data = new get_velocity_list(null);
    // Deserialize message field [velocity_list]
    data.velocity_list = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.velocity_list.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'diffusion_policy/get_velocity_list';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '857cfa79eed99d6668b53660789af2e6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] velocity_list
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new get_velocity_list(null);
    if (msg.velocity_list !== undefined) {
      resolved.velocity_list = msg.velocity_list;
    }
    else {
      resolved.velocity_list = []
    }

    return resolved;
    }
};

module.exports = get_velocity_list;
