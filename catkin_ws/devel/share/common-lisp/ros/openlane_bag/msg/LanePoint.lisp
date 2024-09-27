; Auto-generated. Do not edit!


(cl:in-package openlane_bag-msg)


;//! \htmlinclude LanePoint.msg.html

(cl:defclass <LanePoint> (roslisp-msg-protocol:ros-message)
  ((x
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
   (visibility
    :reader visibility
    :initarg :visibility
    :type cl:float
    :initform 0.0))
)

(cl:defclass LanePoint (<LanePoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LanePoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LanePoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openlane_bag-msg:<LanePoint> is deprecated: use openlane_bag-msg:LanePoint instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <LanePoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openlane_bag-msg:x-val is deprecated.  Use openlane_bag-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <LanePoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openlane_bag-msg:y-val is deprecated.  Use openlane_bag-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <LanePoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openlane_bag-msg:z-val is deprecated.  Use openlane_bag-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'visibility-val :lambda-list '(m))
(cl:defmethod visibility-val ((m <LanePoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openlane_bag-msg:visibility-val is deprecated.  Use openlane_bag-msg:visibility instead.")
  (visibility m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LanePoint>) ostream)
  "Serializes a message object of type '<LanePoint>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'visibility))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LanePoint>) istream)
  "Deserializes a message object of type '<LanePoint>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'visibility) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LanePoint>)))
  "Returns string type for a message object of type '<LanePoint>"
  "openlane_bag/LanePoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LanePoint)))
  "Returns string type for a message object of type 'LanePoint"
  "openlane_bag/LanePoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LanePoint>)))
  "Returns md5sum for a message object of type '<LanePoint>"
  "7e3d90e07dd35cfd897ffc2308933bfc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LanePoint)))
  "Returns md5sum for a message object of type 'LanePoint"
  "7e3d90e07dd35cfd897ffc2308933bfc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LanePoint>)))
  "Returns full string definition for message of type '<LanePoint>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 visibility~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LanePoint)))
  "Returns full string definition for message of type 'LanePoint"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 visibility~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LanePoint>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LanePoint>))
  "Converts a ROS message object to a list"
  (cl:list 'LanePoint
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':visibility (visibility msg))
))
