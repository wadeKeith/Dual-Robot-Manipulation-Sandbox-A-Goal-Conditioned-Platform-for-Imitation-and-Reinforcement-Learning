;; Auto-generated. Do not edit!


(when (boundp 'move_demo::SetJointAngles)
  (if (not (find-package "MOVE_DEMO"))
    (make-package "MOVE_DEMO"))
  (shadow 'SetJointAngles (find-package "MOVE_DEMO")))
(unless (find-package "MOVE_DEMO::SETJOINTANGLES")
  (make-package "MOVE_DEMO::SETJOINTANGLES"))
(unless (find-package "MOVE_DEMO::SETJOINTANGLESREQUEST")
  (make-package "MOVE_DEMO::SETJOINTANGLESREQUEST"))
(unless (find-package "MOVE_DEMO::SETJOINTANGLESRESPONSE")
  (make-package "MOVE_DEMO::SETJOINTANGLESRESPONSE"))

(in-package "ROS")





(defclass move_demo::SetJointAnglesRequest
  :super ros::object
  :slots (_elbow_joint _shoulder_lift_joint _shoulder_pan_joint _wrist_1_joint _wrist_2_joint _wrist_3_joint ))

(defmethod move_demo::SetJointAnglesRequest
  (:init
   (&key
    ((:elbow_joint __elbow_joint) 0.0)
    ((:shoulder_lift_joint __shoulder_lift_joint) 0.0)
    ((:shoulder_pan_joint __shoulder_pan_joint) 0.0)
    ((:wrist_1_joint __wrist_1_joint) 0.0)
    ((:wrist_2_joint __wrist_2_joint) 0.0)
    ((:wrist_3_joint __wrist_3_joint) 0.0)
    )
   (send-super :init)
   (setq _elbow_joint (float __elbow_joint))
   (setq _shoulder_lift_joint (float __shoulder_lift_joint))
   (setq _shoulder_pan_joint (float __shoulder_pan_joint))
   (setq _wrist_1_joint (float __wrist_1_joint))
   (setq _wrist_2_joint (float __wrist_2_joint))
   (setq _wrist_3_joint (float __wrist_3_joint))
   self)
  (:elbow_joint
   (&optional __elbow_joint)
   (if __elbow_joint (setq _elbow_joint __elbow_joint)) _elbow_joint)
  (:shoulder_lift_joint
   (&optional __shoulder_lift_joint)
   (if __shoulder_lift_joint (setq _shoulder_lift_joint __shoulder_lift_joint)) _shoulder_lift_joint)
  (:shoulder_pan_joint
   (&optional __shoulder_pan_joint)
   (if __shoulder_pan_joint (setq _shoulder_pan_joint __shoulder_pan_joint)) _shoulder_pan_joint)
  (:wrist_1_joint
   (&optional __wrist_1_joint)
   (if __wrist_1_joint (setq _wrist_1_joint __wrist_1_joint)) _wrist_1_joint)
  (:wrist_2_joint
   (&optional __wrist_2_joint)
   (if __wrist_2_joint (setq _wrist_2_joint __wrist_2_joint)) _wrist_2_joint)
  (:wrist_3_joint
   (&optional __wrist_3_joint)
   (if __wrist_3_joint (setq _wrist_3_joint __wrist_3_joint)) _wrist_3_joint)
  (:serialization-length
   ()
   (+
    ;; float64 _elbow_joint
    8
    ;; float64 _shoulder_lift_joint
    8
    ;; float64 _shoulder_pan_joint
    8
    ;; float64 _wrist_1_joint
    8
    ;; float64 _wrist_2_joint
    8
    ;; float64 _wrist_3_joint
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _elbow_joint
       (sys::poke _elbow_joint (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _shoulder_lift_joint
       (sys::poke _shoulder_lift_joint (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _shoulder_pan_joint
       (sys::poke _shoulder_pan_joint (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wrist_1_joint
       (sys::poke _wrist_1_joint (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wrist_2_joint
       (sys::poke _wrist_2_joint (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wrist_3_joint
       (sys::poke _wrist_3_joint (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _elbow_joint
     (setq _elbow_joint (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _shoulder_lift_joint
     (setq _shoulder_lift_joint (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _shoulder_pan_joint
     (setq _shoulder_pan_joint (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wrist_1_joint
     (setq _wrist_1_joint (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wrist_2_joint
     (setq _wrist_2_joint (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wrist_3_joint
     (setq _wrist_3_joint (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass move_demo::SetJointAnglesResponse
  :super ros::object
  :slots (_success ))

(defmethod move_demo::SetJointAnglesResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass move_demo::SetJointAngles
  :super ros::object
  :slots ())

(setf (get move_demo::SetJointAngles :md5sum-) "8b430f70f7ff32529e5429ef4104cf78")
(setf (get move_demo::SetJointAngles :datatype-) "move_demo/SetJointAngles")
(setf (get move_demo::SetJointAngles :request) move_demo::SetJointAnglesRequest)
(setf (get move_demo::SetJointAngles :response) move_demo::SetJointAnglesResponse)

(defmethod move_demo::SetJointAnglesRequest
  (:response () (instance move_demo::SetJointAnglesResponse :init)))

(setf (get move_demo::SetJointAnglesRequest :md5sum-) "8b430f70f7ff32529e5429ef4104cf78")
(setf (get move_demo::SetJointAnglesRequest :datatype-) "move_demo/SetJointAnglesRequest")
(setf (get move_demo::SetJointAnglesRequest :definition-)
      "# SetJointAngles.srv
float64 elbow_joint
float64 shoulder_lift_joint
float64 shoulder_pan_joint
float64 wrist_1_joint
float64 wrist_2_joint
float64 wrist_3_joint
---
bool success

")

(setf (get move_demo::SetJointAnglesResponse :md5sum-) "8b430f70f7ff32529e5429ef4104cf78")
(setf (get move_demo::SetJointAnglesResponse :datatype-) "move_demo/SetJointAnglesResponse")
(setf (get move_demo::SetJointAnglesResponse :definition-)
      "# SetJointAngles.srv
float64 elbow_joint
float64 shoulder_lift_joint
float64 shoulder_pan_joint
float64 wrist_1_joint
float64 wrist_2_joint
float64 wrist_3_joint
---
bool success

")



(provide :move_demo/SetJointAngles "8b430f70f7ff32529e5429ef4104cf78")


