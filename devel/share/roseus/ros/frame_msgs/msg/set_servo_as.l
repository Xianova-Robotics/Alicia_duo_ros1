;; Auto-generated. Do not edit!


(when (boundp 'frame_msgs::set_servo_as)
  (if (not (find-package "FRAME_MSGS"))
    (make-package "FRAME_MSGS"))
  (shadow 'set_servo_as (find-package "FRAME_MSGS")))
(unless (find-package "FRAME_MSGS::SET_SERVO_AS")
  (make-package "FRAME_MSGS::SET_SERVO_AS"))

(in-package "ROS")
;;//! \htmlinclude set_servo_as.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass frame_msgs::set_servo_as
  :super ros::object
  :slots (_servo_type_4 _servo_target_angle _servo_target_cycle _gripper_angle _servo_target_angle_u8 _servo_target_cycle_u8 _gripper_angle_u8 ))

(defmethod frame_msgs::set_servo_as
  (:init
   (&key
    ((:servo_type_4 __servo_type_4) (instance std_msgs::Float32 :init))
    ((:servo_target_angle __servo_target_angle) (instance std_msgs::Float32MultiArray :init))
    ((:servo_target_cycle __servo_target_cycle) (instance std_msgs::Float32MultiArray :init))
    ((:gripper_angle __gripper_angle) (instance std_msgs::Float32 :init))
    ((:servo_target_angle_u8 __servo_target_angle_u8) (instance std_msgs::UInt8MultiArray :init))
    ((:servo_target_cycle_u8 __servo_target_cycle_u8) (instance std_msgs::UInt8MultiArray :init))
    ((:gripper_angle_u8 __gripper_angle_u8) (instance std_msgs::UInt8MultiArray :init))
    )
   (send-super :init)
   (setq _servo_type_4 __servo_type_4)
   (setq _servo_target_angle __servo_target_angle)
   (setq _servo_target_cycle __servo_target_cycle)
   (setq _gripper_angle __gripper_angle)
   (setq _servo_target_angle_u8 __servo_target_angle_u8)
   (setq _servo_target_cycle_u8 __servo_target_cycle_u8)
   (setq _gripper_angle_u8 __gripper_angle_u8)
   self)
  (:servo_type_4
   (&rest __servo_type_4)
   (if (keywordp (car __servo_type_4))
       (send* _servo_type_4 __servo_type_4)
     (progn
       (if __servo_type_4 (setq _servo_type_4 (car __servo_type_4)))
       _servo_type_4)))
  (:servo_target_angle
   (&rest __servo_target_angle)
   (if (keywordp (car __servo_target_angle))
       (send* _servo_target_angle __servo_target_angle)
     (progn
       (if __servo_target_angle (setq _servo_target_angle (car __servo_target_angle)))
       _servo_target_angle)))
  (:servo_target_cycle
   (&rest __servo_target_cycle)
   (if (keywordp (car __servo_target_cycle))
       (send* _servo_target_cycle __servo_target_cycle)
     (progn
       (if __servo_target_cycle (setq _servo_target_cycle (car __servo_target_cycle)))
       _servo_target_cycle)))
  (:gripper_angle
   (&rest __gripper_angle)
   (if (keywordp (car __gripper_angle))
       (send* _gripper_angle __gripper_angle)
     (progn
       (if __gripper_angle (setq _gripper_angle (car __gripper_angle)))
       _gripper_angle)))
  (:servo_target_angle_u8
   (&rest __servo_target_angle_u8)
   (if (keywordp (car __servo_target_angle_u8))
       (send* _servo_target_angle_u8 __servo_target_angle_u8)
     (progn
       (if __servo_target_angle_u8 (setq _servo_target_angle_u8 (car __servo_target_angle_u8)))
       _servo_target_angle_u8)))
  (:servo_target_cycle_u8
   (&rest __servo_target_cycle_u8)
   (if (keywordp (car __servo_target_cycle_u8))
       (send* _servo_target_cycle_u8 __servo_target_cycle_u8)
     (progn
       (if __servo_target_cycle_u8 (setq _servo_target_cycle_u8 (car __servo_target_cycle_u8)))
       _servo_target_cycle_u8)))
  (:gripper_angle_u8
   (&rest __gripper_angle_u8)
   (if (keywordp (car __gripper_angle_u8))
       (send* _gripper_angle_u8 __gripper_angle_u8)
     (progn
       (if __gripper_angle_u8 (setq _gripper_angle_u8 (car __gripper_angle_u8)))
       _gripper_angle_u8)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float32 _servo_type_4
    (send _servo_type_4 :serialization-length)
    ;; std_msgs/Float32MultiArray _servo_target_angle
    (send _servo_target_angle :serialization-length)
    ;; std_msgs/Float32MultiArray _servo_target_cycle
    (send _servo_target_cycle :serialization-length)
    ;; std_msgs/Float32 _gripper_angle
    (send _gripper_angle :serialization-length)
    ;; std_msgs/UInt8MultiArray _servo_target_angle_u8
    (send _servo_target_angle_u8 :serialization-length)
    ;; std_msgs/UInt8MultiArray _servo_target_cycle_u8
    (send _servo_target_cycle_u8 :serialization-length)
    ;; std_msgs/UInt8MultiArray _gripper_angle_u8
    (send _gripper_angle_u8 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float32 _servo_type_4
       (send _servo_type_4 :serialize s)
     ;; std_msgs/Float32MultiArray _servo_target_angle
       (send _servo_target_angle :serialize s)
     ;; std_msgs/Float32MultiArray _servo_target_cycle
       (send _servo_target_cycle :serialize s)
     ;; std_msgs/Float32 _gripper_angle
       (send _gripper_angle :serialize s)
     ;; std_msgs/UInt8MultiArray _servo_target_angle_u8
       (send _servo_target_angle_u8 :serialize s)
     ;; std_msgs/UInt8MultiArray _servo_target_cycle_u8
       (send _servo_target_cycle_u8 :serialize s)
     ;; std_msgs/UInt8MultiArray _gripper_angle_u8
       (send _gripper_angle_u8 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float32 _servo_type_4
     (send _servo_type_4 :deserialize buf ptr-) (incf ptr- (send _servo_type_4 :serialization-length))
   ;; std_msgs/Float32MultiArray _servo_target_angle
     (send _servo_target_angle :deserialize buf ptr-) (incf ptr- (send _servo_target_angle :serialization-length))
   ;; std_msgs/Float32MultiArray _servo_target_cycle
     (send _servo_target_cycle :deserialize buf ptr-) (incf ptr- (send _servo_target_cycle :serialization-length))
   ;; std_msgs/Float32 _gripper_angle
     (send _gripper_angle :deserialize buf ptr-) (incf ptr- (send _gripper_angle :serialization-length))
   ;; std_msgs/UInt8MultiArray _servo_target_angle_u8
     (send _servo_target_angle_u8 :deserialize buf ptr-) (incf ptr- (send _servo_target_angle_u8 :serialization-length))
   ;; std_msgs/UInt8MultiArray _servo_target_cycle_u8
     (send _servo_target_cycle_u8 :deserialize buf ptr-) (incf ptr- (send _servo_target_cycle_u8 :serialization-length))
   ;; std_msgs/UInt8MultiArray _gripper_angle_u8
     (send _gripper_angle_u8 :deserialize buf ptr-) (incf ptr- (send _gripper_angle_u8 :serialization-length))
   ;;
   self)
  )

(setf (get frame_msgs::set_servo_as :md5sum-) "9a90f2689947b28e8c629edf3384926d")
(setf (get frame_msgs::set_servo_as :datatype-) "frame_msgs/set_servo_as")
(setf (get frame_msgs::set_servo_as :definition-)
      "std_msgs/Float32 servo_type_4
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
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
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


")



(provide :frame_msgs/set_servo_as "9a90f2689947b28e8c629edf3384926d")


