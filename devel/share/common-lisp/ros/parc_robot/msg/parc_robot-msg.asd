
(cl:in-package :asdf)

(defsystem "parc_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RobotStatus" :depends-on ("_package_RobotStatus"))
    (:file "_package_RobotStatus" :depends-on ("_package"))
    (:file "Weed" :depends-on ("_package_Weed"))
    (:file "_package_Weed" :depends-on ("_package"))
    (:file "WeedDetection" :depends-on ("_package_WeedDetection"))
    (:file "_package_WeedDetection" :depends-on ("_package"))
  ))