
(cl:in-package :asdf)

(defsystem "diffusion_policy-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "cameradata" :depends-on ("_package_cameradata"))
    (:file "_package_cameradata" :depends-on ("_package"))
    (:file "eef_pose" :depends-on ("_package_eef_pose"))
    (:file "_package_eef_pose" :depends-on ("_package"))
    (:file "eef_pose_vel" :depends-on ("_package_eef_pose_vel"))
    (:file "_package_eef_pose_vel" :depends-on ("_package"))
    (:file "get_angles_list" :depends-on ("_package_get_angles_list"))
    (:file "_package_get_angles_list" :depends-on ("_package"))
    (:file "get_velocity_list" :depends-on ("_package_get_velocity_list"))
    (:file "_package_get_velocity_list" :depends-on ("_package"))
    (:file "joint_angles" :depends-on ("_package_joint_angles"))
    (:file "_package_joint_angles" :depends-on ("_package"))
    (:file "joint_vel" :depends-on ("_package_joint_vel"))
    (:file "_package_joint_vel" :depends-on ("_package"))
    (:file "obsdata" :depends-on ("_package_obsdata"))
    (:file "_package_obsdata" :depends-on ("_package"))
  ))