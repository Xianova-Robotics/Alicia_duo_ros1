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

class eef_pose_vel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.eef_pose_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('eef_pose_vel')) {
        this.eef_pose_vel = initObj.eef_pose_vel
      }
      else {
        this.eef_pose_vel = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type eef_pose_vel
    // Serialize message field [eef_pose_vel]
    bufferOffset = _arraySerializer.float32(obj.eef_pose_vel, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type eef_pose_vel
    let len;
    let data = new eef_pose_vel(null);
    // Deserialize message field [eef_pose_vel]
    data.eef_pose_vel = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.eef_pose_vel.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'diffusion_policy/eef_pose_vel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6c78ea9f30900fd93d065f768bfb9ecc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] eef_pose_vel
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new eef_pose_vel(null);
    if (msg.eef_pose_vel !== undefined) {
      resolved.eef_pose_vel = msg.eef_pose_vel;
    }
    else {
      resolved.eef_pose_vel = []
    }

    return resolved;
    }
};

module.exports = eef_pose_vel;
