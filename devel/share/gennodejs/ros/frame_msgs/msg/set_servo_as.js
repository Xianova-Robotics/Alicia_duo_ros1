// Auto-generated. Do not edit!

// (in-package frame_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class set_servo_as {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.servo_type_4 = null;
      this.servo_target_angle = null;
      this.servo_target_cycle = null;
      this.gripper_angle = null;
      this.servo_target_angle_u8 = null;
      this.servo_target_cycle_u8 = null;
      this.gripper_angle_u8 = null;
    }
    else {
      if (initObj.hasOwnProperty('servo_type_4')) {
        this.servo_type_4 = initObj.servo_type_4
      }
      else {
        this.servo_type_4 = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('servo_target_angle')) {
        this.servo_target_angle = initObj.servo_target_angle
      }
      else {
        this.servo_target_angle = new std_msgs.msg.Float32MultiArray();
      }
      if (initObj.hasOwnProperty('servo_target_cycle')) {
        this.servo_target_cycle = initObj.servo_target_cycle
      }
      else {
        this.servo_target_cycle = new std_msgs.msg.Float32MultiArray();
      }
      if (initObj.hasOwnProperty('gripper_angle')) {
        this.gripper_angle = initObj.gripper_angle
      }
      else {
        this.gripper_angle = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('servo_target_angle_u8')) {
        this.servo_target_angle_u8 = initObj.servo_target_angle_u8
      }
      else {
        this.servo_target_angle_u8 = new std_msgs.msg.UInt8MultiArray();
      }
      if (initObj.hasOwnProperty('servo_target_cycle_u8')) {
        this.servo_target_cycle_u8 = initObj.servo_target_cycle_u8
      }
      else {
        this.servo_target_cycle_u8 = new std_msgs.msg.UInt8MultiArray();
      }
      if (initObj.hasOwnProperty('gripper_angle_u8')) {
        this.gripper_angle_u8 = initObj.gripper_angle_u8
      }
      else {
        this.gripper_angle_u8 = new std_msgs.msg.UInt8MultiArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type set_servo_as
    // Serialize message field [servo_type_4]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.servo_type_4, buffer, bufferOffset);
    // Serialize message field [servo_target_angle]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.servo_target_angle, buffer, bufferOffset);
    // Serialize message field [servo_target_cycle]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.servo_target_cycle, buffer, bufferOffset);
    // Serialize message field [gripper_angle]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.gripper_angle, buffer, bufferOffset);
    // Serialize message field [servo_target_angle_u8]
    bufferOffset = std_msgs.msg.UInt8MultiArray.serialize(obj.servo_target_angle_u8, buffer, bufferOffset);
    // Serialize message field [servo_target_cycle_u8]
    bufferOffset = std_msgs.msg.UInt8MultiArray.serialize(obj.servo_target_cycle_u8, buffer, bufferOffset);
    // Serialize message field [gripper_angle_u8]
    bufferOffset = std_msgs.msg.UInt8MultiArray.serialize(obj.gripper_angle_u8, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type set_servo_as
    let len;
    let data = new set_servo_as(null);
    // Deserialize message field [servo_type_4]
    data.servo_type_4 = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [servo_target_angle]
    data.servo_target_angle = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [servo_target_cycle]
    data.servo_target_cycle = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [gripper_angle]
    data.gripper_angle = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [servo_target_angle_u8]
    data.servo_target_angle_u8 = std_msgs.msg.UInt8MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [servo_target_cycle_u8]
    data.servo_target_cycle_u8 = std_msgs.msg.UInt8MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [gripper_angle_u8]
    data.gripper_angle_u8 = std_msgs.msg.UInt8MultiArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.servo_target_angle);
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.servo_target_cycle);
    length += std_msgs.msg.UInt8MultiArray.getMessageSize(object.servo_target_angle_u8);
    length += std_msgs.msg.UInt8MultiArray.getMessageSize(object.servo_target_cycle_u8);
    length += std_msgs.msg.UInt8MultiArray.getMessageSize(object.gripper_angle_u8);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'frame_msgs/set_servo_as';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9a90f2689947b28e8c629edf3384926d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float32 servo_type_4
    std_msgs/Float32MultiArray servo_target_angle
    std_msgs/Float32MultiArray servo_target_cycle
    std_msgs/Float32 gripper_angle
    std_msgs/UInt8MultiArray servo_target_angle_u8
    std_msgs/UInt8MultiArray servo_target_cycle_u8
    std_msgs/UInt8MultiArray gripper_angle_u8
    ================================================================================
    MSG: std_msgs/Float32
    float32 data
    ================================================================================
    MSG: std_msgs/Float32MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float32[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    ================================================================================
    MSG: std_msgs/UInt8MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    uint8[]           data          # array of data
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new set_servo_as(null);
    if (msg.servo_type_4 !== undefined) {
      resolved.servo_type_4 = std_msgs.msg.Float32.Resolve(msg.servo_type_4)
    }
    else {
      resolved.servo_type_4 = new std_msgs.msg.Float32()
    }

    if (msg.servo_target_angle !== undefined) {
      resolved.servo_target_angle = std_msgs.msg.Float32MultiArray.Resolve(msg.servo_target_angle)
    }
    else {
      resolved.servo_target_angle = new std_msgs.msg.Float32MultiArray()
    }

    if (msg.servo_target_cycle !== undefined) {
      resolved.servo_target_cycle = std_msgs.msg.Float32MultiArray.Resolve(msg.servo_target_cycle)
    }
    else {
      resolved.servo_target_cycle = new std_msgs.msg.Float32MultiArray()
    }

    if (msg.gripper_angle !== undefined) {
      resolved.gripper_angle = std_msgs.msg.Float32.Resolve(msg.gripper_angle)
    }
    else {
      resolved.gripper_angle = new std_msgs.msg.Float32()
    }

    if (msg.servo_target_angle_u8 !== undefined) {
      resolved.servo_target_angle_u8 = std_msgs.msg.UInt8MultiArray.Resolve(msg.servo_target_angle_u8)
    }
    else {
      resolved.servo_target_angle_u8 = new std_msgs.msg.UInt8MultiArray()
    }

    if (msg.servo_target_cycle_u8 !== undefined) {
      resolved.servo_target_cycle_u8 = std_msgs.msg.UInt8MultiArray.Resolve(msg.servo_target_cycle_u8)
    }
    else {
      resolved.servo_target_cycle_u8 = new std_msgs.msg.UInt8MultiArray()
    }

    if (msg.gripper_angle_u8 !== undefined) {
      resolved.gripper_angle_u8 = std_msgs.msg.UInt8MultiArray.Resolve(msg.gripper_angle_u8)
    }
    else {
      resolved.gripper_angle_u8 = new std_msgs.msg.UInt8MultiArray()
    }

    return resolved;
    }
};

module.exports = set_servo_as;
