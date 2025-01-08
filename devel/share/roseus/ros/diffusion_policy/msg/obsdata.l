;; Auto-generated. Do not edit!


(when (boundp 'diffusion_policy::obsdata)
  (if (not (find-package "DIFFUSION_POLICY"))
    (make-package "DIFFUSION_POLICY"))
  (shadow 'obsdata (find-package "DIFFUSION_POLICY")))
(unless (find-package "DIFFUSION_POLICY::OBSDATA")
  (make-package "DIFFUSION_POLICY::OBSDATA"))

(in-package "ROS")
;;//! \htmlinclude obsdata.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass diffusion_policy::obsdata
  :super ros::object
  :slots (_header _robot_eef_pose _robot_eef_pose_vel _robot_joint _robot_joint_vel _step_idx ))

(defmethod diffusion_policy::obsdata
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_eef_pose __robot_eef_pose) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:robot_eef_pose_vel __robot_eef_pose_vel) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:robot_joint __robot_joint) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:robot_joint_vel __robot_joint_vel) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:step_idx __step_idx) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_eef_pose __robot_eef_pose)
   (setq _robot_eef_pose_vel __robot_eef_pose_vel)
   (setq _robot_joint __robot_joint)
   (setq _robot_joint_vel __robot_joint_vel)
   (setq _step_idx (round __step_idx))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:robot_eef_pose
   (&optional __robot_eef_pose)
   (if __robot_eef_pose (setq _robot_eef_pose __robot_eef_pose)) _robot_eef_pose)
  (:robot_eef_pose_vel
   (&optional __robot_eef_pose_vel)
   (if __robot_eef_pose_vel (setq _robot_eef_pose_vel __robot_eef_pose_vel)) _robot_eef_pose_vel)
  (:robot_joint
   (&optional __robot_joint)
   (if __robot_joint (setq _robot_joint __robot_joint)) _robot_joint)
  (:robot_joint_vel
   (&optional __robot_joint_vel)
   (if __robot_joint_vel (setq _robot_joint_vel __robot_joint_vel)) _robot_joint_vel)
  (:step_idx
   (&optional __step_idx)
   (if __step_idx (setq _step_idx __step_idx)) _step_idx)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _robot_eef_pose
    (* 8    (length _robot_eef_pose)) 4
    ;; float64[] _robot_eef_pose_vel
    (* 8    (length _robot_eef_pose_vel)) 4
    ;; float64[] _robot_joint
    (* 8    (length _robot_joint)) 4
    ;; float64[] _robot_joint_vel
    (* 8    (length _robot_joint_vel)) 4
    ;; int32 _step_idx
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _robot_eef_pose
     (write-long (length _robot_eef_pose) s)
     (dotimes (i (length _robot_eef_pose))
       (sys::poke (elt _robot_eef_pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _robot_eef_pose_vel
     (write-long (length _robot_eef_pose_vel) s)
     (dotimes (i (length _robot_eef_pose_vel))
       (sys::poke (elt _robot_eef_pose_vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _robot_joint
     (write-long (length _robot_joint) s)
     (dotimes (i (length _robot_joint))
       (sys::poke (elt _robot_joint i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _robot_joint_vel
     (write-long (length _robot_joint_vel) s)
     (dotimes (i (length _robot_joint_vel))
       (sys::poke (elt _robot_joint_vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _step_idx
       (write-long _step_idx s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _robot_eef_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robot_eef_pose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _robot_eef_pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _robot_eef_pose_vel
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robot_eef_pose_vel (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _robot_eef_pose_vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _robot_joint
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robot_joint (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _robot_joint i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _robot_joint_vel
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robot_joint_vel (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _robot_joint_vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32 _step_idx
     (setq _step_idx (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get diffusion_policy::obsdata :md5sum-) "af64483db77873de2449b47fc7837f9c")
(setf (get diffusion_policy::obsdata :datatype-) "diffusion_policy/obsdata")
(setf (get diffusion_policy::obsdata :definition-)
      "std_msgs/Header header
float64[] robot_eef_pose
float64[] robot_eef_pose_vel
float64[] robot_joint
float64[] robot_joint_vel
int32 step_idx


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



(provide :diffusion_policy/obsdata "af64483db77873de2449b47fc7837f9c")


