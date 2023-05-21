; Auto-generated. Do not edit!


(cl:in-package parc_robot-msg)


;//! \htmlinclude Weed.msg.html

(cl:defclass <Weed> (roslisp-msg-protocol:ros-message)
  ((latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0))
)

(cl:defclass Weed (<Weed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Weed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Weed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name parc_robot-msg:<Weed> is deprecated: use parc_robot-msg:Weed instead.")))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <Weed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parc_robot-msg:latitude-val is deprecated.  Use parc_robot-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <Weed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parc_robot-msg:longitude-val is deprecated.  Use parc_robot-msg:longitude instead.")
  (longitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Weed>) ostream)
  "Serializes a message object of type '<Weed>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Weed>) istream)
  "Deserializes a message object of type '<Weed>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Weed>)))
  "Returns string type for a message object of type '<Weed>"
  "parc_robot/Weed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Weed)))
  "Returns string type for a message object of type 'Weed"
  "parc_robot/Weed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Weed>)))
  "Returns md5sum for a message object of type '<Weed>"
  "680c6dc7da65a2421a822205dcbdb600")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Weed)))
  "Returns md5sum for a message object of type 'Weed"
  "680c6dc7da65a2421a822205dcbdb600")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Weed>)))
  "Returns full string definition for message of type '<Weed>"
  (cl:format cl:nil "# A weed in the field~%float64 latitude~%float64 longitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Weed)))
  "Returns full string definition for message of type 'Weed"
  (cl:format cl:nil "# A weed in the field~%float64 latitude~%float64 longitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Weed>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Weed>))
  "Converts a ROS message object to a list"
  (cl:list 'Weed
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
))
