
(cl:in-package :asdf)

(defsystem "frame_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "set_servo_as" :depends-on ("_package_set_servo_as"))
    (:file "_package_set_servo_as" :depends-on ("_package"))
  ))