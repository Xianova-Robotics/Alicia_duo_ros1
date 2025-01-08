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

class cameradata {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.rgb = null;
      this.T = null;
      this.H = null;
      this.W = null;
      this.C = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('rgb')) {
        this.rgb = initObj.rgb
      }
      else {
        this.rgb = [];
      }
      if (initObj.hasOwnProperty('T')) {
        this.T = initObj.T
      }
      else {
        this.T = 0;
      }
      if (initObj.hasOwnProperty('H')) {
        this.H = initObj.H
      }
      else {
        this.H = 0;
      }
      if (initObj.hasOwnProperty('W')) {
        this.W = initObj.W
      }
      else {
        this.W = 0;
      }
      if (initObj.hasOwnProperty('C')) {
        this.C = initObj.C
      }
      else {
        this.C = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cameradata
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [rgb]
    bufferOffset = _arraySerializer.uint8(obj.rgb, buffer, bufferOffset, null);
    // Serialize message field [T]
    bufferOffset = _serializer.uint32(obj.T, buffer, bufferOffset);
    // Serialize message field [H]
    bufferOffset = _serializer.uint32(obj.H, buffer, bufferOffset);
    // Serialize message field [W]
    bufferOffset = _serializer.uint32(obj.W, buffer, bufferOffset);
    // Serialize message field [C]
    bufferOffset = _serializer.uint32(obj.C, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cameradata
    let len;
    let data = new cameradata(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [rgb]
    data.rgb = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [T]
    data.T = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [H]
    data.H = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [W]
    data.W = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [C]
    data.C = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.rgb.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'diffusion_policy/cameradata';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9e2a505638727cb25c53a7697e72a52a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # cameraData.msg
    std_msgs/Header header
    uint8[] rgb          # 扁平化后的图像数据
    uint32 T                # 帧数
    uint32 H                # 图像高度
    uint32 W                # 图像宽度
    uint32 C                # 颜色通道数
    
    
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
    const resolved = new cameradata(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.rgb !== undefined) {
      resolved.rgb = msg.rgb;
    }
    else {
      resolved.rgb = []
    }

    if (msg.T !== undefined) {
      resolved.T = msg.T;
    }
    else {
      resolved.T = 0
    }

    if (msg.H !== undefined) {
      resolved.H = msg.H;
    }
    else {
      resolved.H = 0
    }

    if (msg.W !== undefined) {
      resolved.W = msg.W;
    }
    else {
      resolved.W = 0
    }

    if (msg.C !== undefined) {
      resolved.C = msg.C;
    }
    else {
      resolved.C = 0
    }

    return resolved;
    }
};

module.exports = cameradata;
