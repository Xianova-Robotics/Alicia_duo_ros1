// Auto-generated. Do not edit!

// (in-package diffusion_policy.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class obsdata {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.robot_eef_pose = null;
      this.robot_eef_pose_vel = null;
      this.robot_joint = null;
      this.robot_joint_vel = null;
      this.step_idx = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('robot_eef_pose')) {
        this.robot_eef_pose = initObj.robot_eef_pose
      }
      else {
        this.robot_eef_pose = [];
      }
      if (initObj.hasOwnProperty('robot_eef_pose_vel')) {
        this.robot_eef_pose_vel = initObj.robot_eef_pose_vel
      }
      else {
        this.robot_eef_pose_vel = [];
      }
      if (initObj.hasOwnProperty('robot_joint')) {
        this.robot_joint = initObj.robot_joint
      }
      else {
        this.robot_joint = [];
      }
      if (initObj.hasOwnProperty('robot_joint_vel')) {
        this.robot_joint_vel = initObj.robot_joint_vel
      }
      else {
        this.robot_joint_vel = [];
      }
      if (initObj.hasOwnProperty('step_idx')) {
        this.step_idx = initObj.step_idx
      }
      else {
        this.step_idx = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type obsdata
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [robot_eef_pose]
    bufferOffset = _arraySerializer.float64(obj.robot_eef_pose, buffer, bufferOffset, null);
    // Serialize message field [robot_eef_pose_vel]
    bufferOffset = _arraySerializer.float64(obj.robot_eef_pose_vel, buffer, bufferOffset, null);
    // Serialize message field [robot_joint]
    bufferOffset = _arraySerializer.float64(obj.robot_joint, buffer, bufferOffset, null);
    // Serialize message field [robot_joint_vel]
    bufferOffset = _arraySerializer.float64(obj.robot_joint_vel, buffer, bufferOffset, null);
    // Serialize message field [step_idx]
    bufferOffset = _serializer.int32(obj.step_idx, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type obsdata
    let len;
    let data = new obsdata(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [robot_eef_pose]
    data.robot_eef_pose = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [robot_eef_pose_vel]
    data.robot_eef_pose_vel = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [robot_joint]
    data.robot_joint = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [robot_joint_vel]
    data.robot_joint_vel = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [step_idx]
    data.step_idx = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.robot_eef_pose.length;
    length += 8 * object.robot_eef_pose_vel.length;
    length += 8 * object.robot_joint.length;
    length += 8 * object.robot_joint_vel.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'diffusion_policy/obsdata';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'af64483db77873de2449b47fc7837f9c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    float64[] robot_eef_pose
    float64[] robot_eef_pose_vel
    float64[] robot_joint
    float64[] robot_joint_vel
    int32 step_idx
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new obsdata(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.robot_eef_pose !== undefined) {
      resolved.robot_eef_pose = msg.robot_eef_pose;
    }
    else {
      resolved.robot_eef_pose = []
    }

    if (msg.robot_eef_pose_vel !== undefined) {
      resolved.robot_eef_pose_vel = msg.robot_eef_pose_vel;
    }
    else {
      resolved.robot_eef_pose_vel = []
    }

    if (msg.robot_joint !== undefined) {
      resolved.robot_joint = msg.robot_joint;
    }
    else {
      resolved.robot_joint = []
    }

    if (msg.robot_joint_vel !== undefined) {
      resolved.robot_joint_vel = msg.robot_joint_vel;
    }
    else {
      resolved.robot_joint_vel = []
    }

    if (msg.step_idx !== undefined) {
      resolved.step_idx = msg.step_idx;
    }
    else {
      resolved.step_idx = 0
    }

    return resolved;
    }
};

module.exports = obsdata;
