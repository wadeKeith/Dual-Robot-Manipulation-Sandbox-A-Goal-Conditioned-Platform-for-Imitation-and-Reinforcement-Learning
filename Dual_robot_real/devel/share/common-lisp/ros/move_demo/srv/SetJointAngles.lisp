; Auto-generated. Do not edit!


(cl:in-package move_demo-srv)


;//! \htmlinclude SetJointAngles-request.msg.html

(cl:defclass <SetJointAngles-request> (roslisp-msg-protocol:ros-message)
  ((elbow_joint
    :reader elbow_joint
    :initarg :elbow_joint
    :type cl:float
    :initform 0.0)
   (shoulder_lift_joint
    :reader shoulder_lift_joint
    :initarg :shoulder_lift_joint
    :type cl:float
    :initform 0.0)
   (shoulder_pan_joint
    :reader shoulder_pan_joint
    :initarg :shoulder_pan_joint
    :type cl:float
    :initform 0.0)
   (wrist_1_joint
    :reader wrist_1_joint
    :initarg :wrist_1_joint
    :type cl:float
    :initform 0.0)
   (wrist_2_joint
    :reader wrist_2_joint
    :initarg :wrist_2_joint
    :type cl:float
    :initform 0.0)
   (wrist_3_joint
    :reader wrist_3_joint
    :initarg :wrist_3_joint
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetJointAngles-request (<SetJointAngles-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointAngles-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointAngles-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_demo-srv:<SetJointAngles-request> is deprecated: use move_demo-srv:SetJointAngles-request instead.")))

(cl:ensure-generic-function 'elbow_joint-val :lambda-list '(m))
(cl:defmethod elbow_joint-val ((m <SetJointAngles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_demo-srv:elbow_joint-val is deprecated.  Use move_demo-srv:elbow_joint instead.")
  (elbow_joint m))

(cl:ensure-generic-function 'shoulder_lift_joint-val :lambda-list '(m))
(cl:defmethod shoulder_lift_joint-val ((m <SetJointAngles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_demo-srv:shoulder_lift_joint-val is deprecated.  Use move_demo-srv:shoulder_lift_joint instead.")
  (shoulder_lift_joint m))

(cl:ensure-generic-function 'shoulder_pan_joint-val :lambda-list '(m))
(cl:defmethod shoulder_pan_joint-val ((m <SetJointAngles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_demo-srv:shoulder_pan_joint-val is deprecated.  Use move_demo-srv:shoulder_pan_joint instead.")
  (shoulder_pan_joint m))

(cl:ensure-generic-function 'wrist_1_joint-val :lambda-list '(m))
(cl:defmethod wrist_1_joint-val ((m <SetJointAngles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_demo-srv:wrist_1_joint-val is deprecated.  Use move_demo-srv:wrist_1_joint instead.")
  (wrist_1_joint m))

(cl:ensure-generic-function 'wrist_2_joint-val :lambda-list '(m))
(cl:defmethod wrist_2_joint-val ((m <SetJointAngles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_demo-srv:wrist_2_joint-val is deprecated.  Use move_demo-srv:wrist_2_joint instead.")
  (wrist_2_joint m))

(cl:ensure-generic-function 'wrist_3_joint-val :lambda-list '(m))
(cl:defmethod wrist_3_joint-val ((m <SetJointAngles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_demo-srv:wrist_3_joint-val is deprecated.  Use move_demo-srv:wrist_3_joint instead.")
  (wrist_3_joint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointAngles-request>) ostream)
  "Serializes a message object of type '<SetJointAngles-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'elbow_joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'shoulder_lift_joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'shoulder_pan_joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wrist_1_joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wrist_2_joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wrist_3_joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointAngles-request>) istream)
  "Deserializes a message object of type '<SetJointAngles-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'elbow_joint) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'shoulder_lift_joint) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'shoulder_pan_joint) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wrist_1_joint) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wrist_2_joint) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wrist_3_joint) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointAngles-request>)))
  "Returns string type for a service object of type '<SetJointAngles-request>"
  "move_demo/SetJointAnglesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointAngles-request)))
  "Returns string type for a service object of type 'SetJointAngles-request"
  "move_demo/SetJointAnglesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointAngles-request>)))
  "Returns md5sum for a message object of type '<SetJointAngles-request>"
  "8b430f70f7ff32529e5429ef4104cf78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointAngles-request)))
  "Returns md5sum for a message object of type 'SetJointAngles-request"
  "8b430f70f7ff32529e5429ef4104cf78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointAngles-request>)))
  "Returns full string definition for message of type '<SetJointAngles-request>"
  (cl:format cl:nil "# SetJointAngles.srv~%float64 elbow_joint~%float64 shoulder_lift_joint~%float64 shoulder_pan_joint~%float64 wrist_1_joint~%float64 wrist_2_joint~%float64 wrist_3_joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointAngles-request)))
  "Returns full string definition for message of type 'SetJointAngles-request"
  (cl:format cl:nil "# SetJointAngles.srv~%float64 elbow_joint~%float64 shoulder_lift_joint~%float64 shoulder_pan_joint~%float64 wrist_1_joint~%float64 wrist_2_joint~%float64 wrist_3_joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointAngles-request>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointAngles-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointAngles-request
    (cl:cons ':elbow_joint (elbow_joint msg))
    (cl:cons ':shoulder_lift_joint (shoulder_lift_joint msg))
    (cl:cons ':shoulder_pan_joint (shoulder_pan_joint msg))
    (cl:cons ':wrist_1_joint (wrist_1_joint msg))
    (cl:cons ':wrist_2_joint (wrist_2_joint msg))
    (cl:cons ':wrist_3_joint (wrist_3_joint msg))
))
;//! \htmlinclude SetJointAngles-response.msg.html

(cl:defclass <SetJointAngles-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetJointAngles-response (<SetJointAngles-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointAngles-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointAngles-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_demo-srv:<SetJointAngles-response> is deprecated: use move_demo-srv:SetJointAngles-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetJointAngles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_demo-srv:success-val is deprecated.  Use move_demo-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointAngles-response>) ostream)
  "Serializes a message object of type '<SetJointAngles-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointAngles-response>) istream)
  "Deserializes a message object of type '<SetJointAngles-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointAngles-response>)))
  "Returns string type for a service object of type '<SetJointAngles-response>"
  "move_demo/SetJointAnglesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointAngles-response)))
  "Returns string type for a service object of type 'SetJointAngles-response"
  "move_demo/SetJointAnglesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointAngles-response>)))
  "Returns md5sum for a message object of type '<SetJointAngles-response>"
  "8b430f70f7ff32529e5429ef4104cf78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointAngles-response)))
  "Returns md5sum for a message object of type 'SetJointAngles-response"
  "8b430f70f7ff32529e5429ef4104cf78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointAngles-response>)))
  "Returns full string definition for message of type '<SetJointAngles-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointAngles-response)))
  "Returns full string definition for message of type 'SetJointAngles-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointAngles-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointAngles-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointAngles-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetJointAngles)))
  'SetJointAngles-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetJointAngles)))
  'SetJointAngles-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointAngles)))
  "Returns string type for a service object of type '<SetJointAngles>"
  "move_demo/SetJointAngles")