; Auto-generated. Do not edit!


(cl:in-package frame_msgs-msg)


;//! \htmlinclude set_servo_as.msg.html

(cl:defclass <set_servo_as> (roslisp-msg-protocol:ros-message)
  ((servo_target_angle
    :reader servo_target_angle
    :initarg :servo_target_angle
    :type std_msgs-msg:Float32MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float32MultiArray))
   (servo_target_cycle
    :reader servo_target_cycle
    :initarg :servo_target_cycle
    :type std_msgs-msg:Float32MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float32MultiArray))
   (servo_target_angle_u8
    :reader servo_target_angle_u8
    :initarg :servo_target_angle_u8
    :type std_msgs-msg:UInt8MultiArray
    :initform (cl:make-instance 'std_msgs-msg:UInt8MultiArray))
   (servo_target_cycle_u8
    :reader servo_target_cycle_u8
    :initarg :servo_target_cycle_u8
    :type std_msgs-msg:UInt8MultiArray
    :initform (cl:make-instance 'std_msgs-msg:UInt8MultiArray)))
)

(cl:defclass set_servo_as (<set_servo_as>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_servo_as>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_servo_as)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name frame_msgs-msg:<set_servo_as> is deprecated: use frame_msgs-msg:set_servo_as instead.")))

(cl:ensure-generic-function 'servo_target_angle-val :lambda-list '(m))
(cl:defmethod servo_target_angle-val ((m <set_servo_as>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader frame_msgs-msg:servo_target_angle-val is deprecated.  Use frame_msgs-msg:servo_target_angle instead.")
  (servo_target_angle m))

(cl:ensure-generic-function 'servo_target_cycle-val :lambda-list '(m))
(cl:defmethod servo_target_cycle-val ((m <set_servo_as>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader frame_msgs-msg:servo_target_cycle-val is deprecated.  Use frame_msgs-msg:servo_target_cycle instead.")
  (servo_target_cycle m))

(cl:ensure-generic-function 'servo_target_angle_u8-val :lambda-list '(m))
(cl:defmethod servo_target_angle_u8-val ((m <set_servo_as>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader frame_msgs-msg:servo_target_angle_u8-val is deprecated.  Use frame_msgs-msg:servo_target_angle_u8 instead.")
  (servo_target_angle_u8 m))

(cl:ensure-generic-function 'servo_target_cycle_u8-val :lambda-list '(m))
(cl:defmethod servo_target_cycle_u8-val ((m <set_servo_as>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader frame_msgs-msg:servo_target_cycle_u8-val is deprecated.  Use frame_msgs-msg:servo_target_cycle_u8 instead.")
  (servo_target_cycle_u8 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_servo_as>) ostream)
  "Serializes a message object of type '<set_servo_as>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'servo_target_angle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'servo_target_cycle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'servo_target_angle_u8) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'servo_target_cycle_u8) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_servo_as>) istream)
  "Deserializes a message object of type '<set_servo_as>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'servo_target_angle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'servo_target_cycle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'servo_target_angle_u8) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'servo_target_cycle_u8) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_servo_as>)))
  "Returns string type for a message object of type '<set_servo_as>"
  "frame_msgs/set_servo_as")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_servo_as)))
  "Returns string type for a message object of type 'set_servo_as"
  "frame_msgs/set_servo_as")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_servo_as>)))
  "Returns md5sum for a message object of type '<set_servo_as>"
  "47ff6d7a480da9dc74da49bb9bec5dcf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_servo_as)))
  "Returns md5sum for a message object of type 'set_servo_as"
  "47ff6d7a480da9dc74da49bb9bec5dcf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_servo_as>)))
  "Returns full string definition for message of type '<set_servo_as>"
  (cl:format cl:nil "std_msgs/Float32MultiArray servo_target_angle~%std_msgs/Float32MultiArray servo_target_cycle~%std_msgs/UInt8MultiArray servo_target_angle_u8~%std_msgs/UInt8MultiArray servo_target_cycle_u8~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/UInt8MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%uint8[]           data          # array of data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_servo_as)))
  "Returns full string definition for message of type 'set_servo_as"
  (cl:format cl:nil "std_msgs/Float32MultiArray servo_target_angle~%std_msgs/Float32MultiArray servo_target_cycle~%std_msgs/UInt8MultiArray servo_target_angle_u8~%std_msgs/UInt8MultiArray servo_target_cycle_u8~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/UInt8MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%uint8[]           data          # array of data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_servo_as>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'servo_target_angle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'servo_target_cycle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'servo_target_angle_u8))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'servo_target_cycle_u8))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_servo_as>))
  "Converts a ROS message object to a list"
  (cl:list 'set_servo_as
    (cl:cons ':servo_target_angle (servo_target_angle msg))
    (cl:cons ':servo_target_cycle (servo_target_cycle msg))
    (cl:cons ':servo_target_angle_u8 (servo_target_angle_u8 msg))
    (cl:cons ':servo_target_cycle_u8 (servo_target_cycle_u8 msg))
))
