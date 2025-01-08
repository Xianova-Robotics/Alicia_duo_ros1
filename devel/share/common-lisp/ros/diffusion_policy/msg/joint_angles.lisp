; Auto-generated. Do not edit!


(cl:in-package diffusion_policy-msg)


;//! \htmlinclude joint_angles.msg.html

(cl:defclass <joint_angles> (roslisp-msg-protocol:ros-message)
  ((joint_angles
    :reader joint_angles
    :initarg :joint_angles
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass joint_angles (<joint_angles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joint_angles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joint_angles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name diffusion_policy-msg:<joint_angles> is deprecated: use diffusion_policy-msg:joint_angles instead.")))

(cl:ensure-generic-function 'joint_angles-val :lambda-list '(m))
(cl:defmethod joint_angles-val ((m <joint_angles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader diffusion_policy-msg:joint_angles-val is deprecated.  Use diffusion_policy-msg:joint_angles instead.")
  (joint_angles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joint_angles>) ostream)
  "Serializes a message object of type '<joint_angles>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_angles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_angles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joint_angles>) istream)
  "Deserializes a message object of type '<joint_angles>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_angles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_angles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<joint_angles>)))
  "Returns string type for a message object of type '<joint_angles>"
  "diffusion_policy/joint_angles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joint_angles)))
  "Returns string type for a message object of type 'joint_angles"
  "diffusion_policy/joint_angles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<joint_angles>)))
  "Returns md5sum for a message object of type '<joint_angles>"
  "11501c45f507c225d25f998a0b6418cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joint_angles)))
  "Returns md5sum for a message object of type 'joint_angles"
  "11501c45f507c225d25f998a0b6418cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joint_angles>)))
  "Returns full string definition for message of type '<joint_angles>"
  (cl:format cl:nil "float32[] joint_angles~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joint_angles)))
  "Returns full string definition for message of type 'joint_angles"
  (cl:format cl:nil "float32[] joint_angles~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joint_angles>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_angles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joint_angles>))
  "Converts a ROS message object to a list"
  (cl:list 'joint_angles
    (cl:cons ':joint_angles (joint_angles msg))
))
