; Auto-generated. Do not edit!


(cl:in-package diffusion_policy-msg)


;//! \htmlinclude get_velocity_list.msg.html

(cl:defclass <get_velocity_list> (roslisp-msg-protocol:ros-message)
  ((velocity_list
    :reader velocity_list
    :initarg :velocity_list
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass get_velocity_list (<get_velocity_list>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_velocity_list>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_velocity_list)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name diffusion_policy-msg:<get_velocity_list> is deprecated: use diffusion_policy-msg:get_velocity_list instead.")))

(cl:ensure-generic-function 'velocity_list-val :lambda-list '(m))
(cl:defmethod velocity_list-val ((m <get_velocity_list>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader diffusion_policy-msg:velocity_list-val is deprecated.  Use diffusion_policy-msg:velocity_list instead.")
  (velocity_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_velocity_list>) ostream)
  "Serializes a message object of type '<get_velocity_list>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'velocity_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'velocity_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_velocity_list>) istream)
  "Deserializes a message object of type '<get_velocity_list>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'velocity_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'velocity_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_velocity_list>)))
  "Returns string type for a message object of type '<get_velocity_list>"
  "diffusion_policy/get_velocity_list")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_velocity_list)))
  "Returns string type for a message object of type 'get_velocity_list"
  "diffusion_policy/get_velocity_list")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_velocity_list>)))
  "Returns md5sum for a message object of type '<get_velocity_list>"
  "857cfa79eed99d6668b53660789af2e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_velocity_list)))
  "Returns md5sum for a message object of type 'get_velocity_list"
  "857cfa79eed99d6668b53660789af2e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_velocity_list>)))
  "Returns full string definition for message of type '<get_velocity_list>"
  (cl:format cl:nil "float32[] velocity_list~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_velocity_list)))
  "Returns full string definition for message of type 'get_velocity_list"
  (cl:format cl:nil "float32[] velocity_list~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_velocity_list>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'velocity_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_velocity_list>))
  "Converts a ROS message object to a list"
  (cl:list 'get_velocity_list
    (cl:cons ':velocity_list (velocity_list msg))
))
