;; Auto-generated. Do not edit!


(when (boundp 'diffusion_policy::eef_pose)
  (if (not (find-package "DIFFUSION_POLICY"))
    (make-package "DIFFUSION_POLICY"))
  (shadow 'eef_pose (find-package "DIFFUSION_POLICY")))
(unless (find-package "DIFFUSION_POLICY::EEF_POSE")
  (make-package "DIFFUSION_POLICY::EEF_POSE"))

(in-package "ROS")
;;//! \htmlinclude eef_pose.msg.html


(defclass diffusion_policy::eef_pose
  :super ros::object
  :slots (_eef_pose ))

(defmethod diffusion_policy::eef_pose
  (:init
   (&key
    ((:eef_pose __eef_pose) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _eef_pose __eef_pose)
   self)
  (:eef_pose
   (&optional __eef_pose)
   (if __eef_pose (setq _eef_pose __eef_pose)) _eef_pose)
  (:serialization-length
   ()
   (+
    ;; float32[] _eef_pose
    (* 4    (length _eef_pose)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _eef_pose
     (write-long (length _eef_pose) s)
     (dotimes (i (length _eef_pose))
       (sys::poke (elt _eef_pose i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _eef_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _eef_pose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _eef_pose i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get diffusion_policy::eef_pose :md5sum-) "3dada35c3813c36ce3dd435fc2d196ca")
(setf (get diffusion_policy::eef_pose :datatype-) "diffusion_policy/eef_pose")
(setf (get diffusion_policy::eef_pose :definition-)
      "float32[] eef_pose
")



(provide :diffusion_policy/eef_pose "3dada35c3813c36ce3dd435fc2d196ca")


