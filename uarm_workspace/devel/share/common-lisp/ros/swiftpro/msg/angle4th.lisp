; Auto-generated. Do not edit!


(cl:in-package swiftpro-msg)


;//! \htmlinclude angle4th.msg.html

(cl:defclass <angle4th> (roslisp-msg-protocol:ros-message)
  ((angle4th
    :reader angle4th
    :initarg :angle4th
    :type cl:float
    :initform 0.0))
)

(cl:defclass angle4th (<angle4th>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <angle4th>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'angle4th)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name swiftpro-msg:<angle4th> is deprecated: use swiftpro-msg:angle4th instead.")))

(cl:ensure-generic-function 'angle4th-val :lambda-list '(m))
(cl:defmethod angle4th-val ((m <angle4th>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader swiftpro-msg:angle4th-val is deprecated.  Use swiftpro-msg:angle4th instead.")
  (angle4th m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <angle4th>) ostream)
  "Serializes a message object of type '<angle4th>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle4th))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <angle4th>) istream)
  "Deserializes a message object of type '<angle4th>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle4th) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<angle4th>)))
  "Returns string type for a message object of type '<angle4th>"
  "swiftpro/angle4th")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'angle4th)))
  "Returns string type for a message object of type 'angle4th"
  "swiftpro/angle4th")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<angle4th>)))
  "Returns md5sum for a message object of type '<angle4th>"
  "8eecd591854543ff5e9cf583de2d05e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'angle4th)))
  "Returns md5sum for a message object of type 'angle4th"
  "8eecd591854543ff5e9cf583de2d05e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<angle4th>)))
  "Returns full string definition for message of type '<angle4th>"
  (cl:format cl:nil "float64 angle4th~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'angle4th)))
  "Returns full string definition for message of type 'angle4th"
  (cl:format cl:nil "float64 angle4th~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <angle4th>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <angle4th>))
  "Converts a ROS message object to a list"
  (cl:list 'angle4th
    (cl:cons ':angle4th (angle4th msg))
))
