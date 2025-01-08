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

class joint_vel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_vel')) {
        this.joint_vel = initObj.joint_vel
      }
      else {
        this.joint_vel = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type joint_vel
    // Serialize message field [joint_vel]
    bufferOffset = _arraySerializer.float32(obj.joint_vel, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type joint_vel
    let len;
    let data = new joint_vel(null);
    // Deserialize message field [joint_vel]
    data.joint_vel = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.joint_vel.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'diffusion_policy/joint_vel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '735276c7cfd280519eb976cd7d752b51';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] joint_vel
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new joint_vel(null);
    if (msg.joint_vel !== undefined) {
      resolved.joint_vel = msg.joint_vel;
    }
    else {
      resolved.joint_vel = []
    }

    return resolved;
    }
};

module.exports = joint_vel;
