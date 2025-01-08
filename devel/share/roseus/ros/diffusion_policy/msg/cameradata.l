;; Auto-generated. Do not edit!


(when (boundp 'diffusion_policy::cameradata)
  (if (not (find-package "DIFFUSION_POLICY"))
    (make-package "DIFFUSION_POLICY"))
  (shadow 'cameradata (find-package "DIFFUSION_POLICY")))
(unless (find-package "DIFFUSION_POLICY::CAMERADATA")
  (make-package "DIFFUSION_POLICY::CAMERADATA"))

(in-package "ROS")
;;//! \htmlinclude cameradata.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass diffusion_policy::cameradata
  :super ros::object
  :slots (_header _rgb _T _H _W _C ))

(defmethod diffusion_policy::cameradata
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rgb __rgb) (make-array 0 :initial-element 0 :element-type :char))
    ((:T __T) 0)
    ((:H __H) 0)
    ((:W __W) 0)
    ((:C __C) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _rgb __rgb)
   (setq _T (round __T))
   (setq _H (round __H))
   (setq _W (round __W))
   (setq _C (round __C))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:rgb
   (&optional __rgb)
   (if __rgb (setq _rgb __rgb)) _rgb)
  (:T
   (&optional __T)
   (if __T (setq _T __T)) _T)
  (:H
   (&optional __H)
   (if __H (setq _H __H)) _H)
  (:W
   (&optional __W)
   (if __W (setq _W __W)) _W)
  (:C
   (&optional __C)
   (if __C (setq _C __C)) _C)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8[] _rgb
    (* 1    (length _rgb)) 4
    ;; uint32 _T
    4
    ;; uint32 _H
    4
    ;; uint32 _W
    4
    ;; uint32 _C
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8[] _rgb
     (write-long (length _rgb) s)
     (princ _rgb s)
     ;; uint32 _T
       (write-long _T s)
     ;; uint32 _H
       (write-long _H s)
     ;; uint32 _W
       (write-long _W s)
     ;; uint32 _C
       (write-long _C s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8[] _rgb
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _rgb (make-array n :element-type :char))
     (replace _rgb buf :start2 ptr-) (incf ptr- n))
   ;; uint32 _T
     (setq _T (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _H
     (setq _H (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _W
     (setq _W (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _C
     (setq _C (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get diffusion_policy::cameradata :md5sum-) "9e2a505638727cb25c53a7697e72a52a")
(setf (get diffusion_policy::cameradata :datatype-) "diffusion_policy/cameradata")
(setf (get diffusion_policy::cameradata :definition-)
      "# cameraData.msg
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

")



(provide :diffusion_policy/cameradata "9e2a505638727cb25c53a7697e72a52a")


