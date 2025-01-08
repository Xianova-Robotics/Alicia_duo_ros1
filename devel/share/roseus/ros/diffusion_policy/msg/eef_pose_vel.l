;; Auto-generated. Do not edit!


(when (boundp 'diffusion_policy::eef_pose_vel)
  (if (not (find-package "DIFFUSION_POLICY"))
    (make-package "DIFFUSION_POLICY"))
  (shadow 'eef_pose_vel (find-package "DIFFUSION_POLICY")))
(unless (find-package "DIFFUSION_POLICY::EEF_POSE_VEL")
  (make-package "DIFFUSION_POLICY::EEF_POSE_VEL"))

(in-package "ROS")
;;//! \htmlinclude eef_pose_vel.msg.html


(defclass diffusion_policy::eef_pose_vel
  :super ros::object
  :slots (_eef_pose_vel ))

(defmethod diffusion_policy::eef_pose_vel
  (:init
   (&key
    ((:eef_pose_vel __eef_pose_vel) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _eef_pose_vel __eef_pose_vel)
   self)
  (:eef_pose_vel
   (&optional __eef_pose_vel)
   (if __eef_pose_vel (setq _eef_pose_vel __eef_pose_vel)) _eef_pose_vel)
  (:serialization-length
   ()
   (+
    ;; float32[] _eef_pose_vel
    (* 4    (length _eef_pose_vel)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _eef_pose_vel
     (write-long (length _eef_pose_vel) s)
     (dotimes (i (length _eef_pose_vel))
       (sys::poke (elt _eef_pose_vel i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _eef_pose_vel
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _eef_pose_vel (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _eef_pose_vel i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get diffusion_policy::eef_pose_vel :md5sum-) "6c78ea9f30900fd93d065f768bfb9ecc")
(setf (get diffusion_policy::eef_pose_vel :datatype-) "diffusion_policy/eef_pose_vel")
(setf (get diffusion_policy::eef_pose_vel :definition-)
      "float32[] eef_pose_vel
")



(provide :diffusion_policy/eef_pose_vel "6c78ea9f30900fd93d065f768bfb9ecc")


