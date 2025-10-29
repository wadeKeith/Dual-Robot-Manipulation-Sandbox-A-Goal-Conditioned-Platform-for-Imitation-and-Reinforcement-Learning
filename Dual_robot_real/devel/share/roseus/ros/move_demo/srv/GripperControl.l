;; Auto-generated. Do not edit!


(when (boundp 'move_demo::GripperControl)
  (if (not (find-package "MOVE_DEMO"))
    (make-package "MOVE_DEMO"))
  (shadow 'GripperControl (find-package "MOVE_DEMO")))
(unless (find-package "MOVE_DEMO::GRIPPERCONTROL")
  (make-package "MOVE_DEMO::GRIPPERCONTROL"))
(unless (find-package "MOVE_DEMO::GRIPPERCONTROLREQUEST")
  (make-package "MOVE_DEMO::GRIPPERCONTROLREQUEST"))
(unless (find-package "MOVE_DEMO::GRIPPERCONTROLRESPONSE")
  (make-package "MOVE_DEMO::GRIPPERCONTROLRESPONSE"))

(in-package "ROS")





(defclass move_demo::GripperControlRequest
  :super ros::object
  :slots (_width ))

(defmethod move_demo::GripperControlRequest
  (:init
   (&key
    ((:width __width) 0.0)
    )
   (send-super :init)
   (setq _width (float __width))
   self)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:serialization-length
   ()
   (+
    ;; float32 _width
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _width
       (sys::poke _width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _width
     (setq _width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass move_demo::GripperControlResponse
  :super ros::object
  :slots (_success ))

(defmethod move_demo::GripperControlResponse
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

(defclass move_demo::GripperControl
  :super ros::object
  :slots ())

(setf (get move_demo::GripperControl :md5sum-) "be5ef003f22cb11bc267a44b935e1625")
(setf (get move_demo::GripperControl :datatype-) "move_demo/GripperControl")
(setf (get move_demo::GripperControl :request) move_demo::GripperControlRequest)
(setf (get move_demo::GripperControl :response) move_demo::GripperControlResponse)

(defmethod move_demo::GripperControlRequest
  (:response () (instance move_demo::GripperControlResponse :init)))

(setf (get move_demo::GripperControlRequest :md5sum-) "be5ef003f22cb11bc267a44b935e1625")
(setf (get move_demo::GripperControlRequest :datatype-) "move_demo/GripperControlRequest")
(setf (get move_demo::GripperControlRequest :definition-)
      "float32 width
---
bool success
")

(setf (get move_demo::GripperControlResponse :md5sum-) "be5ef003f22cb11bc267a44b935e1625")
(setf (get move_demo::GripperControlResponse :datatype-) "move_demo/GripperControlResponse")
(setf (get move_demo::GripperControlResponse :definition-)
      "float32 width
---
bool success
")



(provide :move_demo/GripperControl "be5ef003f22cb11bc267a44b935e1625")


