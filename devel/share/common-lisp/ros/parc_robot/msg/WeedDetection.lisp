; Auto-generated. Do not edit!


(cl:in-package parc_robot-msg)


;//! \htmlinclude WeedDetection.msg.html

(cl:defclass <WeedDetection> (roslisp-msg-protocol:ros-message)
  ((weeds
    :reader weeds
    :initarg :weeds
    :type (cl:vector parc_robot-msg:Weed)
   :initform (cl:make-array 0 :element-type 'parc_robot-msg:Weed :initial-element (cl:make-instance 'parc_robot-msg:Weed))))
)

(cl:defclass WeedDetection (<WeedDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WeedDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WeedDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name parc_robot-msg:<WeedDetection> is deprecated: use parc_robot-msg:WeedDetection instead.")))

(cl:ensure-generic-function 'weeds-val :lambda-list '(m))
(cl:defmethod weeds-val ((m <WeedDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader parc_robot-msg:weeds-val is deprecated.  Use parc_robot-msg:weeds instead.")
  (weeds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WeedDetection>) ostream)
  "Serializes a message object of type '<WeedDetection>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'weeds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'weeds))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WeedDetection>) istream)
  "Deserializes a message object of type '<WeedDetection>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'weeds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'weeds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'parc_robot-msg:Weed))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WeedDetection>)))
  "Returns string type for a message object of type '<WeedDetection>"
  "parc_robot/WeedDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WeedDetection)))
  "Returns string type for a message object of type 'WeedDetection"
  "parc_robot/WeedDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WeedDetection>)))
  "Returns md5sum for a message object of type '<WeedDetection>"
  "845a4e3a48d0573bc750910e15745135")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WeedDetection)))
  "Returns md5sum for a message object of type 'WeedDetection"
  "845a4e3a48d0573bc750910e15745135")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WeedDetection>)))
  "Returns full string definition for message of type '<WeedDetection>"
  (cl:format cl:nil "# A list of detected weeds~%Weed[] weeds~%~%~%================================================================================~%MSG: parc_robot/Weed~%# A weed in the field~%float64 latitude~%float64 longitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WeedDetection)))
  "Returns full string definition for message of type 'WeedDetection"
  (cl:format cl:nil "# A list of detected weeds~%Weed[] weeds~%~%~%================================================================================~%MSG: parc_robot/Weed~%# A weed in the field~%float64 latitude~%float64 longitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WeedDetection>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'weeds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WeedDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'WeedDetection
    (cl:cons ':weeds (weeds msg))
))
