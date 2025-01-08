;; Auto-generated. Do not edit!


(when (boundp 'diffusion_policy::get_angles_list)
  (if (not (find-package "DIFFUSION_POLICY"))
    (make-package "DIFFUSION_POLICY"))
  (shadow 'get_angles_list (find-package "DIFFUSION_POLICY")))
(unless (find-package "DIFFUSION_POLICY::GET_ANGLES_LIST")
  (make-package "DIFFUSION_POLICY::GET_ANGLES_LIST"))

(in-package "ROS")
;;//! \htmlinclude get_angles_list.msg.html


(defclass diffusion_policy::get_angles_list
  :super ros::object
  :slots (_angle_list ))

(defmethod diffusion_policy::get_angles_list
  (:init
   (&key
    ((:angle_list __angle_list) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _angle_list __angle_list)
   self)
  (:angle_list
   (&optional __angle_list)
   (if __angle_list (setq _angle_list __angle_list)) _angle_list)
  (:serialization-length
   ()
   (+
    ;; float32[] _angle_list
    (* 4    (length _angle_list)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _angle_list
     (write-long (length _angle_list) s)
     (dotimes (i (length _angle_list))
       (sys::poke (elt _angle_list i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _angle_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angle_list (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _angle_list i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get diffusion_policy::get_angles_list :md5sum-) "39fd64490c91d853399586438fc1152b")
(setf (get diffusion_policy::get_angles_list :datatype-) "diffusion_policy/get_angles_list")
(setf (get diffusion_policy::get_angles_list :definition-)
      "float32[] angle_list

")



(provide :diffusion_policy/get_angles_list "39fd64490c91d853399586438fc1152b")


