; Auto-generated. Do not edit!


(cl:in-package swiftpro-msg)


;//! \htmlinclude SwiftproState.msg.html

(cl:defclass <SwiftproState> (roslisp-msg-protocol:ros-message)
  ((motor_angle1
    :reader motor_angle1
    :initarg :motor_angle1
    :type cl:float
    :initform 0.0)
   (motor_angle2
    :reader motor_angle2
    :initarg :motor_angle2
    :type cl:float
    :initform 0.0)
   (motor_angle3
    :reader motor_angle3
    :initarg :motor_angle3
    :type cl:float
    :initform 0.0)
   (motor_angle4
    :reader motor_angle4
    :initarg :motor_angle4
    :type cl:float
    :initform 0.0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (pump
    :reader pump
    :initarg :pump
    :type cl:fixnum
    :initform 0)
   (swiftpro_status
    :reader swiftpro_status
    :initarg :swiftpro_status
    :type cl:fixnum
    :initform 0)
   (gripper
    :reader gripper
    :initarg :gripper
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SwiftproState (<SwiftproState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwiftproState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwiftproState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swiftpro-msg:<SwiftproState> is deprecated: use swiftpro-msg:SwiftproState instead.")))

(cl:ensure-generic-function 'motor_angle1-val :lambda-list '(m))
(cl:defmethod motor_angle1-val ((m <SwiftproState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swiftpro-msg:motor_angle1-val is deprecated.  Use swiftpro-msg:motor_angle1 instead.")
  (motor_angle1 m))

(cl:ensure-generic-function 'motor_angle2-val :lambda-list '(m))
(cl:defmethod motor_angle2-val ((m <SwiftproState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swiftpro-msg:motor_angle2-val is deprecated.  Use swiftpro-msg:motor_angle2 instead.")
  (motor_angle2 m))

(cl:ensure-generic-function 'motor_angle3-val :lambda-list '(m))
(cl:defmethod motor_angle3-val ((m <SwiftproState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swiftpro-msg:motor_angle3-val is deprecated.  Use swiftpro-msg:motor_angle3 instead.")
  (motor_angle3 m))

(cl:ensure-generic-function 'motor_angle4-val :lambda-list '(m))
(cl:defmethod motor_angle4-val ((m <SwiftproState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swiftpro-msg:motor_angle4-val is deprecated.  Use swiftpro-msg:motor_angle4 instead.")
  (motor_angle4 m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <SwiftproState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swiftpro-msg:x-val is deprecated.  Use swiftpro-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <SwiftproState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swiftpro-msg:y-val is deprecated.  Use swiftpro-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <SwiftproState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swiftpro-msg:z-val is deprecated.  Use swiftpro-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'pump-val :lambda-list '(m))
(cl:defmethod pump-val ((m <SwiftproState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swiftpro-msg:pump-val is deprecated.  Use swiftpro-msg:pump instead.")
  (pump m))

(cl:ensure-generic-function 'swiftpro_status-val :lambda-list '(m))
(cl:defmethod swiftpro_status-val ((m <SwiftproState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swiftpro-msg:swiftpro_status-val is deprecated.  Use swiftpro-msg:swiftpro_status instead.")
  (swiftpro_status m))

(cl:ensure-generic-function 'gripper-val :lambda-list '(m))
(cl:defmethod gripper-val ((m <SwiftproState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swiftpro-msg:gripper-val is deprecated.  Use swiftpro-msg:gripper instead.")
  (gripper m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwiftproState>) ostream)
  "Serializes a message object of type '<SwiftproState>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'motor_angle1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'motor_angle2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'motor_angle3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'motor_angle4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pump)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swiftpro_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gripper)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwiftproState>) istream)
  "Deserializes a message object of type '<SwiftproState>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor_angle1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor_angle2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor_angle3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor_angle4) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pump)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swiftpro_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gripper)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwiftproState>)))
  "Returns string type for a message object of type '<SwiftproState>"
  "swiftpro/SwiftproState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwiftproState)))
  "Returns string type for a message object of type 'SwiftproState"
  "swiftpro/SwiftproState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwiftproState>)))
  "Returns md5sum for a message object of type '<SwiftproState>"
  "bcd9671f860a15ba5765d673098d21bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwiftproState)))
  "Returns md5sum for a message object of type 'SwiftproState"
  "bcd9671f860a15ba5765d673098d21bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwiftproState>)))
  "Returns full string definition for message of type '<SwiftproState>"
  (cl:format cl:nil "float64 motor_angle1~%float64 motor_angle2~%float64 motor_angle3~%float64 motor_angle4~%float64 x~%float64 y~%float64 z~%uint8 	pump~%uint8 	swiftpro_status~%uint8 	gripper~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwiftproState)))
  "Returns full string definition for message of type 'SwiftproState"
  (cl:format cl:nil "float64 motor_angle1~%float64 motor_angle2~%float64 motor_angle3~%float64 motor_angle4~%float64 x~%float64 y~%float64 z~%uint8 	pump~%uint8 	swiftpro_status~%uint8 	gripper~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwiftproState>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwiftproState>))
  "Converts a ROS message object to a list"
  (cl:list 'SwiftproState
    (cl:cons ':motor_angle1 (motor_angle1 msg))
    (cl:cons ':motor_angle2 (motor_angle2 msg))
    (cl:cons ':motor_angle3 (motor_angle3 msg))
    (cl:cons ':motor_angle4 (motor_angle4 msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':pump (pump msg))
    (cl:cons ':swiftpro_status (swiftpro_status msg))
    (cl:cons ':gripper (gripper msg))
))
