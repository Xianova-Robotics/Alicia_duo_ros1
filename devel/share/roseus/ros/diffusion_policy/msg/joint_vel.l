;; Auto-generated. Do not edit!


(when (boundp 'diffusion_policy::joint_vel)
  (if (not (find-package "DIFFUSION_POLICY"))
    (make-package "DIFFUSION_POLICY"))
  (shadow 'joint_vel (find-package "DIFFUSION_POLICY")))
(unless (find-package "DIFFUSION_POLICY::JOINT_VEL")
  (make-package "DIFFUSION_POLICY::JOINT_VEL"))

(in-package "ROS")
;;//! \htmlinclude joint_vel.msg.html


(defclass diffusion_policy::joint_vel
  :super ros::object
  :slots (_joint_vel ))

(defmethod diffusion_policy::joint_vel
  (:init
   (&key
    ((:joint_vel __joint_vel) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joint_vel __joint_vel)
   self)
  (:joint_vel
   (&optional __joint_vel)
   (if __joint_vel (setq _joint_vel __joint_vel)) _joint_vel)
  (:serialization-length
   ()
   (+
    ;; float32[] _joint_vel
    (* 4    (length _joint_vel)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _joint_vel
     (write-long (length _joint_vel) s)
     (dotimes (i (length _joint_vel))
       (sys::poke (elt _joint_vel i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _joint_vel
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_vel (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_vel i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get diffusion_policy::joint_vel :md5sum-) "735276c7cfd280519eb976cd7d752b51")
(setf (get diffusion_policy::joint_vel :datatype-) "diffusion_policy/joint_vel")
(setf (get diffusion_policy::joint_vel :definition-)
      "float32[] joint_vel
")



(provide :diffusion_policy/joint_vel "735276c7cfd280519eb976cd7d752b51")


