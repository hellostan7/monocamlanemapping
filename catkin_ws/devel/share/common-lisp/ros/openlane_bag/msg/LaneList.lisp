; Auto-generated. Do not edit!


(cl:in-package openlane_bag-msg)


;//! \htmlinclude LaneList.msg.html

(cl:defclass <LaneList> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lane_list
    :reader lane_list
    :initarg :lane_list
    :type (cl:vector openlane_bag-msg:Lane)
   :initform (cl:make-array 0 :element-type 'openlane_bag-msg:Lane :initial-element (cl:make-instance 'openlane_bag-msg:Lane)))
   (num_lanes
    :reader num_lanes
    :initarg :num_lanes
    :type cl:integer
    :initform 0))
)

(cl:defclass LaneList (<LaneList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LaneList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LaneList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name openlane_bag-msg:<LaneList> is deprecated: use openlane_bag-msg:LaneList instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LaneList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openlane_bag-msg:header-val is deprecated.  Use openlane_bag-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lane_list-val :lambda-list '(m))
(cl:defmethod lane_list-val ((m <LaneList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openlane_bag-msg:lane_list-val is deprecated.  Use openlane_bag-msg:lane_list instead.")
  (lane_list m))

(cl:ensure-generic-function 'num_lanes-val :lambda-list '(m))
(cl:defmethod num_lanes-val ((m <LaneList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader openlane_bag-msg:num_lanes-val is deprecated.  Use openlane_bag-msg:num_lanes instead.")
  (num_lanes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LaneList>) ostream)
  "Serializes a message object of type '<LaneList>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lane_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'lane_list))
  (cl:let* ((signed (cl:slot-value msg 'num_lanes)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LaneList>) istream)
  "Deserializes a message object of type '<LaneList>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lane_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lane_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'openlane_bag-msg:Lane))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_lanes) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LaneList>)))
  "Returns string type for a message object of type '<LaneList>"
  "openlane_bag/LaneList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LaneList)))
  "Returns string type for a message object of type 'LaneList"
  "openlane_bag/LaneList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LaneList>)))
  "Returns md5sum for a message object of type '<LaneList>"
  "7c734ba2a0461f54efa410edc16ce01f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LaneList)))
  "Returns md5sum for a message object of type 'LaneList"
  "7c734ba2a0461f54efa410edc16ce01f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LaneList>)))
  "Returns full string definition for message of type '<LaneList>"
  (cl:format cl:nil "Header header~%Lane[] lane_list~%int32 num_lanes~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: openlane_bag/Lane~%LanePoint[] lane~%int32 num_points~%int32 attribute  # attribute: the left-left lane is 1, the left lane is 2, the right lane is 3, and the right-right lane is 4.~%int32 track_id~%int32 category~%~%================================================================================~%MSG: openlane_bag/LanePoint~%float32 x~%float32 y~%float32 z~%float32 visibility~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LaneList)))
  "Returns full string definition for message of type 'LaneList"
  (cl:format cl:nil "Header header~%Lane[] lane_list~%int32 num_lanes~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: openlane_bag/Lane~%LanePoint[] lane~%int32 num_points~%int32 attribute  # attribute: the left-left lane is 1, the left lane is 2, the right lane is 3, and the right-right lane is 4.~%int32 track_id~%int32 category~%~%================================================================================~%MSG: openlane_bag/LanePoint~%float32 x~%float32 y~%float32 z~%float32 visibility~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LaneList>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lane_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LaneList>))
  "Converts a ROS message object to a list"
  (cl:list 'LaneList
    (cl:cons ':header (header msg))
    (cl:cons ':lane_list (lane_list msg))
    (cl:cons ':num_lanes (num_lanes msg))
))
