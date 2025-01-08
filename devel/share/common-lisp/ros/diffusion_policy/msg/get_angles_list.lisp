; Auto-generated. Do not edit!


(cl:in-package diffusion_policy-msg)


;//! \htmlinclude get_angles_list.msg.html

(cl:defclass <get_angles_list> (roslisp-msg-protocol:ros-message)
  ((angle_list
    :reader angle_list
    :initarg :angle_list
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass get_angles_list (<get_angles_list>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_angles_list>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_angles_list)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name diffusion_policy-msg:<get_angles_list> is deprecated: use diffusion_policy-msg:get_angles_list instead.")))

(cl:ensure-generic-function 'angle_list-val :lambda-list '(m))
(cl:defmethod angle_list-val ((m <get_angles_list>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader diffusion_policy-msg:angle_list-val is deprecated.  Use diffusion_policy-msg:angle_list instead.")
  (angle_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_angles_list>) ostream)
  "Serializes a message object of type '<get_angles_list>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'angle_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'angle_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_angles_list>) istream)
  "Deserializes a message object of type '<get_angles_list>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'angle_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'angle_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_angles_list>)))
  "Returns string type for a message object of type '<get_angles_list>"
  "diffusion_policy/get_angles_list")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_angles_list)))
  "Returns string type for a message object of type 'get_angles_list"
  "diffusion_policy/get_angles_list")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_angles_list>)))
  "Returns md5sum for a message object of type '<get_angles_list>"
  "39fd64490c91d853399586438fc1152b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_angles_list)))
  "Returns md5sum for a message object of type 'get_angles_list"
  "39fd64490c91d853399586438fc1152b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_angles_list>)))
  "Returns full string definition for message of type '<get_angles_list>"
  (cl:format cl:nil "float32[] angle_list~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_angles_list)))
  "Returns full string definition for message of type 'get_angles_list"
  (cl:format cl:nil "float32[] angle_list~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_angles_list>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'angle_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_angles_list>))
  "Converts a ROS message object to a list"
  (cl:list 'get_angles_list
    (cl:cons ':angle_list (angle_list msg))
))
