; Auto-generated. Do not edit!


(cl:in-package diffusion_policy-msg)


;//! \htmlinclude eef_pose_vel.msg.html

(cl:defclass <eef_pose_vel> (roslisp-msg-protocol:ros-message)
  ((eef_pose_vel
    :reader eef_pose_vel
    :initarg :eef_pose_vel
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass eef_pose_vel (<eef_pose_vel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <eef_pose_vel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'eef_pose_vel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name diffusion_policy-msg:<eef_pose_vel> is deprecated: use diffusion_policy-msg:eef_pose_vel instead.")))

(cl:ensure-generic-function 'eef_pose_vel-val :lambda-list '(m))
(cl:defmethod eef_pose_vel-val ((m <eef_pose_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader diffusion_policy-msg:eef_pose_vel-val is deprecated.  Use diffusion_policy-msg:eef_pose_vel instead.")
  (eef_pose_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <eef_pose_vel>) ostream)
  "Serializes a message object of type '<eef_pose_vel>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'eef_pose_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'eef_pose_vel))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <eef_pose_vel>) istream)
  "Deserializes a message object of type '<eef_pose_vel>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'eef_pose_vel) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'eef_pose_vel)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<eef_pose_vel>)))
  "Returns string type for a message object of type '<eef_pose_vel>"
  "diffusion_policy/eef_pose_vel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'eef_pose_vel)))
  "Returns string type for a message object of type 'eef_pose_vel"
  "diffusion_policy/eef_pose_vel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<eef_pose_vel>)))
  "Returns md5sum for a message object of type '<eef_pose_vel>"
  "6c78ea9f30900fd93d065f768bfb9ecc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'eef_pose_vel)))
  "Returns md5sum for a message object of type 'eef_pose_vel"
  "6c78ea9f30900fd93d065f768bfb9ecc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<eef_pose_vel>)))
  "Returns full string definition for message of type '<eef_pose_vel>"
  (cl:format cl:nil "float32[] eef_pose_vel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'eef_pose_vel)))
  "Returns full string definition for message of type 'eef_pose_vel"
  (cl:format cl:nil "float32[] eef_pose_vel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <eef_pose_vel>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'eef_pose_vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <eef_pose_vel>))
  "Converts a ROS message object to a list"
  (cl:list 'eef_pose_vel
    (cl:cons ':eef_pose_vel (eef_pose_vel msg))
))
