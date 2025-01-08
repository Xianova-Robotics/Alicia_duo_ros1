;; Auto-generated. Do not edit!


(when (boundp 'diffusion_policy::joint_angles)
  (if (not (find-package "DIFFUSION_POLICY"))
    (make-package "DIFFUSION_POLICY"))
  (shadow 'joint_angles (find-package "DIFFUSION_POLICY")))
(unless (find-package "DIFFUSION_POLICY::JOINT_ANGLES")
  (make-package "DIFFUSION_POLICY::JOINT_ANGLES"))

(in-package "ROS")
;;//! \htmlinclude joint_angles.msg.html


(defclass diffusion_policy::joint_angles
  :super ros::object
  :slots (_joint_angles ))

(defmethod diffusion_policy::joint_angles
  (:init
   (&key
    ((:joint_angles __joint_angles) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joint_angles __joint_angles)
   self)
  (:joint_angles
   (&optional __joint_angles)
   (if __joint_angles (setq _joint_angles __joint_angles)) _joint_angles)
  (:serialization-length
   ()
   (+
    ;; float32[] _joint_angles
    (* 4    (length _joint_angles)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _joint_angles
     (write-long (length _joint_angles) s)
     (dotimes (i (length _joint_angles))
       (sys::poke (elt _joint_angles i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _joint_angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_angles (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_angles i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get diffusion_policy::joint_angles :md5sum-) "11501c45f507c225d25f998a0b6418cd")
(setf (get diffusion_policy::joint_angles :datatype-) "diffusion_policy/joint_angles")
(setf (get diffusion_policy::joint_angles :definition-)
      "float32[] joint_angles
")



(provide :diffusion_policy/joint_angles "11501c45f507c225d25f998a0b6418cd")


