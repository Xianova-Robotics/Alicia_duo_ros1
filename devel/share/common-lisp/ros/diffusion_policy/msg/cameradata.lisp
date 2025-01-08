; Auto-generated. Do not edit!


(cl:in-package diffusion_policy-msg)


;//! \htmlinclude cameradata.msg.html

(cl:defclass <cameradata> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (rgb
    :reader rgb
    :initarg :rgb
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (T
    :reader T
    :initarg :T
    :type cl:integer
    :initform 0)
   (H
    :reader H
    :initarg :H
    :type cl:integer
    :initform 0)
   (W
    :reader W
    :initarg :W
    :type cl:integer
    :initform 0)
   (C
    :reader C
    :initarg :C
    :type cl:integer
    :initform 0))
)

(cl:defclass cameradata (<cameradata>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cameradata>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cameradata)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name diffusion_policy-msg:<cameradata> is deprecated: use diffusion_policy-msg:cameradata instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <cameradata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader diffusion_policy-msg:header-val is deprecated.  Use diffusion_policy-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'rgb-val :lambda-list '(m))
(cl:defmethod rgb-val ((m <cameradata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader diffusion_policy-msg:rgb-val is deprecated.  Use diffusion_policy-msg:rgb instead.")
  (rgb m))

(cl:ensure-generic-function 'T-val :lambda-list '(m))
(cl:defmethod T-val ((m <cameradata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader diffusion_policy-msg:T-val is deprecated.  Use diffusion_policy-msg:T instead.")
  (T m))

(cl:ensure-generic-function 'H-val :lambda-list '(m))
(cl:defmethod H-val ((m <cameradata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader diffusion_policy-msg:H-val is deprecated.  Use diffusion_policy-msg:H instead.")
  (H m))

(cl:ensure-generic-function 'W-val :lambda-list '(m))
(cl:defmethod W-val ((m <cameradata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader diffusion_policy-msg:W-val is deprecated.  Use diffusion_policy-msg:W instead.")
  (W m))

(cl:ensure-generic-function 'C-val :lambda-list '(m))
(cl:defmethod C-val ((m <cameradata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader diffusion_policy-msg:C-val is deprecated.  Use diffusion_policy-msg:C instead.")
  (C m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cameradata>) ostream)
  "Serializes a message object of type '<cameradata>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rgb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'rgb))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'T)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'T)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'T)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'T)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'H)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'H)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'H)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'H)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'W)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'W)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'W)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'W)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'C)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'C)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'C)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'C)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cameradata>) istream)
  "Deserializes a message object of type '<cameradata>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rgb) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rgb)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'T)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'T)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'T)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'T)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'H)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'H)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'H)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'H)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'W)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'W)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'W)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'W)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'C)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'C)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'C)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'C)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cameradata>)))
  "Returns string type for a message object of type '<cameradata>"
  "diffusion_policy/cameradata")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cameradata)))
  "Returns string type for a message object of type 'cameradata"
  "diffusion_policy/cameradata")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cameradata>)))
  "Returns md5sum for a message object of type '<cameradata>"
  "9e2a505638727cb25c53a7697e72a52a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cameradata)))
  "Returns md5sum for a message object of type 'cameradata"
  "9e2a505638727cb25c53a7697e72a52a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cameradata>)))
  "Returns full string definition for message of type '<cameradata>"
  (cl:format cl:nil "# cameraData.msg~%std_msgs/Header header~%uint8[] rgb          # 扁平化后的图像数据~%uint32 T                # 帧数~%uint32 H                # 图像高度~%uint32 W                # 图像宽度~%uint32 C                # 颜色通道数~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cameradata)))
  "Returns full string definition for message of type 'cameradata"
  (cl:format cl:nil "# cameraData.msg~%std_msgs/Header header~%uint8[] rgb          # 扁平化后的图像数据~%uint32 T                # 帧数~%uint32 H                # 图像高度~%uint32 W                # 图像宽度~%uint32 C                # 颜色通道数~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cameradata>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rgb) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cameradata>))
  "Converts a ROS message object to a list"
  (cl:list 'cameradata
    (cl:cons ':header (header msg))
    (cl:cons ':rgb (rgb msg))
    (cl:cons ':T (T msg))
    (cl:cons ':H (H msg))
    (cl:cons ':W (W msg))
    (cl:cons ':C (C msg))
))
