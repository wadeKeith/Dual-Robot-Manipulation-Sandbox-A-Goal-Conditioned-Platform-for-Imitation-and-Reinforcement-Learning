
(cl:in-package :asdf)

(defsystem "move_demo-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GripperControl" :depends-on ("_package_GripperControl"))
    (:file "_package_GripperControl" :depends-on ("_package"))
    (:file "SetJointAngles" :depends-on ("_package_SetJointAngles"))
    (:file "_package_SetJointAngles" :depends-on ("_package"))
  ))