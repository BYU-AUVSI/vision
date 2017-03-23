; Auto-generated. Do not edit!


(cl:in-package sniper_cam-msg)


;//! \htmlinclude interopImages.msg.html

(cl:defclass <interopImages> (roslisp-msg-protocol:ros-message)
  ((gps_lat
    :reader gps_lat
    :initarg :gps_lat
    :type cl:float
    :initform 0.0))
)

(cl:defclass interopImages (<interopImages>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <interopImages>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'interopImages)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sniper_cam-msg:<interopImages> is deprecated: use sniper_cam-msg:interopImages instead.")))

(cl:ensure-generic-function 'gps_lat-val :lambda-list '(m))
(cl:defmethod gps_lat-val ((m <interopImages>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sniper_cam-msg:gps_lat-val is deprecated.  Use sniper_cam-msg:gps_lat instead.")
  (gps_lat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <interopImages>) ostream)
  "Serializes a message object of type '<interopImages>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gps_lat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <interopImages>) istream)
  "Deserializes a message object of type '<interopImages>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gps_lat) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<interopImages>)))
  "Returns string type for a message object of type '<interopImages>"
  "sniper_cam/interopImages")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'interopImages)))
  "Returns string type for a message object of type 'interopImages"
  "sniper_cam/interopImages")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<interopImages>)))
  "Returns md5sum for a message object of type '<interopImages>"
  "be350a6f6b980def99e12724f4fb21de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'interopImages)))
  "Returns md5sum for a message object of type 'interopImages"
  "be350a6f6b980def99e12724f4fb21de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<interopImages>)))
  "Returns full string definition for message of type '<interopImages>"
  (cl:format cl:nil "float32 gps_lat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'interopImages)))
  "Returns full string definition for message of type 'interopImages"
  (cl:format cl:nil "float32 gps_lat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <interopImages>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <interopImages>))
  "Converts a ROS message object to a list"
  (cl:list 'interopImages
    (cl:cons ':gps_lat (gps_lat msg))
))
