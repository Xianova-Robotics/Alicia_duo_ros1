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

class get_angles_list {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle_list = null;
    }
    else {
      if (initObj.hasOwnProperty('angle_list')) {
        this.angle_list = initObj.angle_list
      }
      else {
        this.angle_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type get_angles_list
    // Serialize message field [angle_list]
    bufferOffset = _arraySerializer.float32(obj.angle_list, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type get_angles_list
    let len;
    let data = new get_angles_list(null);
    // Deserialize message field [angle_list]
    data.angle_list = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.angle_list.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'diffusion_policy/get_angles_list';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '39fd64490c91d853399586438fc1152b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] angle_list
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new get_angles_list(null);
    if (msg.angle_list !== undefined) {
      resolved.angle_list = msg.angle_list;
    }
    else {
      resolved.angle_list = []
    }

    return resolved;
    }
};

module.exports = get_angles_list;
