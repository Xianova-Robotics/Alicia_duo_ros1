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

class eef_pose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.eef_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('eef_pose')) {
        this.eef_pose = initObj.eef_pose
      }
      else {
        this.eef_pose = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type eef_pose
    // Serialize message field [eef_pose]
    bufferOffset = _arraySerializer.float32(obj.eef_pose, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type eef_pose
    let len;
    let data = new eef_pose(null);
    // Deserialize message field [eef_pose]
    data.eef_pose = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.eef_pose.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'diffusion_policy/eef_pose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3dada35c3813c36ce3dd435fc2d196ca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] eef_pose
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new eef_pose(null);
    if (msg.eef_pose !== undefined) {
      resolved.eef_pose = msg.eef_pose;
    }
    else {
      resolved.eef_pose = []
    }

    return resolved;
    }
};

module.exports = eef_pose;
