;; Auto-generated. Do not edit!


(when (boundp 'diffusion_policy::get_velocity_list)
  (if (not (find-package "DIFFUSION_POLICY"))
    (make-package "DIFFUSION_POLICY"))
  (shadow 'get_velocity_list (find-package "DIFFUSION_POLICY")))
(unless (find-package "DIFFUSION_POLICY::GET_VELOCITY_LIST")
  (make-package "DIFFUSION_POLICY::GET_VELOCITY_LIST"))

(in-package "ROS")
;;//! \htmlinclude get_velocity_list.msg.html


(defclass diffusion_policy::get_velocity_list
  :super ros::object
  :slots (_velocity_list ))

(defmethod diffusion_policy::get_velocity_list
  (:init
   (&key
    ((:velocity_list __velocity_list) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _velocity_list __velocity_list)
   self)
  (:velocity_list
   (&optional __velocity_list)
   (if __velocity_list (setq _velocity_list __velocity_list)) _velocity_list)
  (:serialization-length
   ()
   (+
    ;; float32[] _velocity_list
    (* 4    (length _velocity_list)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _velocity_list
     (write-long (length _velocity_list) s)
     (dotimes (i (length _velocity_list))
       (sys::poke (elt _velocity_list i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _velocity_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity_list (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocity_list i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get diffusion_policy::get_velocity_list :md5sum-) "857cfa79eed99d6668b53660789af2e6")
(setf (get diffusion_policy::get_velocity_list :datatype-) "diffusion_policy/get_velocity_list")
(setf (get diffusion_policy::get_velocity_list :definition-)
      "float32[] velocity_list
")



(provide :diffusion_policy/get_velocity_list "857cfa79eed99d6668b53660789af2e6")


