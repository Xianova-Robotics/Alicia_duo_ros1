; Auto-generated. Do not edit!


(cl:in-package diffusion_policy-msg)


;//! \htmlinclude eef_pose.msg.html

(cl:defclass <eef_pose> (roslisp-msg-protocol:ros-message)
  ((eef_pose
    :reader eef_pose
    :initarg :eef_pose
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass eef_pose (<eef_pose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <eef_pose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'eef_pose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name diffusion_policy-msg:<eef_pose> is deprecated: use diffusion_policy-msg:eef_pose instead.")))

(cl:ensure-generic-function 'eef_pose-val :lambda-list '(m))
(cl:defmethod eef_pose-val ((m <eef_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader diffusion_policy-msg:eef_pose-val is deprecated.  Use diffusion_policy-msg:eef_pose instead.")
  (eef_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <eef_pose>) ostream)
  "Serializes a message object of type '<eef_pose>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'eef_pose))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'eef_pose))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <eef_pose>) istream)
  "Deserializes a message object of type '<eef_pose>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'eef_pose) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'eef_pose)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<eef_pose>)))
  "Returns string type for a message object of type '<eef_pose>"
  "diffusion_policy/eef_pose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'eef_pose)))
  "Returns string type for a message object of type 'eef_pose"
  "diffusion_policy/eef_pose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<eef_pose>)))
  "Returns md5sum for a message object of type '<eef_pose>"
  "3dada35c3813c36ce3dd435fc2d196ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'eef_pose)))
  "Returns md5sum for a message object of type 'eef_pose"
  "3dada35c3813c36ce3dd435fc2d196ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<eef_pose>)))
  "Returns full string definition for message of type '<eef_pose>"
  (cl:format cl:nil "float32[] eef_pose~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'eef_pose)))
  "Returns full string definition for message of type 'eef_pose"
  (cl:format cl:nil "float32[] eef_pose~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <eef_pose>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'eef_pose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <eef_pose>))
  "Converts a ROS message object to a list"
  (cl:list 'eef_pose
    (cl:cons ':eef_pose (eef_pose msg))
))
