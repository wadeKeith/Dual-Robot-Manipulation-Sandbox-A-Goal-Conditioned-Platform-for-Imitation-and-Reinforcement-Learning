; Auto-generated. Do not edit!


(cl:in-package move_demo-srv)


;//! \htmlinclude GripperControl-request.msg.html

(cl:defclass <GripperControl-request> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0))
)

(cl:defclass GripperControl-request (<GripperControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_demo-srv:<GripperControl-request> is deprecated: use move_demo-srv:GripperControl-request instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <GripperControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_demo-srv:width-val is deprecated.  Use move_demo-srv:width instead.")
  (width m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperControl-request>) ostream)
  "Serializes a message object of type '<GripperControl-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperControl-request>) istream)
  "Deserializes a message object of type '<GripperControl-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperControl-request>)))
  "Returns string type for a service object of type '<GripperControl-request>"
  "move_demo/GripperControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperControl-request)))
  "Returns string type for a service object of type 'GripperControl-request"
  "move_demo/GripperControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperControl-request>)))
  "Returns md5sum for a message object of type '<GripperControl-request>"
  "be5ef003f22cb11bc267a44b935e1625")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperControl-request)))
  "Returns md5sum for a message object of type 'GripperControl-request"
  "be5ef003f22cb11bc267a44b935e1625")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperControl-request>)))
  "Returns full string definition for message of type '<GripperControl-request>"
  (cl:format cl:nil "float32 width~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperControl-request)))
  "Returns full string definition for message of type 'GripperControl-request"
  (cl:format cl:nil "float32 width~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperControl-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperControl-request
    (cl:cons ':width (width msg))
))
;//! \htmlinclude GripperControl-response.msg.html

(cl:defclass <GripperControl-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GripperControl-response (<GripperControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_demo-srv:<GripperControl-response> is deprecated: use move_demo-srv:GripperControl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GripperControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_demo-srv:success-val is deprecated.  Use move_demo-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperControl-response>) ostream)
  "Serializes a message object of type '<GripperControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperControl-response>) istream)
  "Deserializes a message object of type '<GripperControl-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperControl-response>)))
  "Returns string type for a service object of type '<GripperControl-response>"
  "move_demo/GripperControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperControl-response)))
  "Returns string type for a service object of type 'GripperControl-response"
  "move_demo/GripperControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperControl-response>)))
  "Returns md5sum for a message object of type '<GripperControl-response>"
  "be5ef003f22cb11bc267a44b935e1625")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperControl-response)))
  "Returns md5sum for a message object of type 'GripperControl-response"
  "be5ef003f22cb11bc267a44b935e1625")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperControl-response>)))
  "Returns full string definition for message of type '<GripperControl-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperControl-response)))
  "Returns full string definition for message of type 'GripperControl-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperControl-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperControl-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GripperControl)))
  'GripperControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GripperControl)))
  'GripperControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperControl)))
  "Returns string type for a service object of type '<GripperControl>"
  "move_demo/GripperControl")