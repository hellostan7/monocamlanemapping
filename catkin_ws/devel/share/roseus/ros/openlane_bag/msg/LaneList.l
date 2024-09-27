;; Auto-generated. Do not edit!


(when (boundp 'openlane_bag::LaneList)
  (if (not (find-package "OPENLANE_BAG"))
    (make-package "OPENLANE_BAG"))
  (shadow 'LaneList (find-package "OPENLANE_BAG")))
(unless (find-package "OPENLANE_BAG::LANELIST")
  (make-package "OPENLANE_BAG::LANELIST"))

(in-package "ROS")
;;//! \htmlinclude LaneList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass openlane_bag::LaneList
  :super ros::object
  :slots (_header _lane_list _num_lanes ))

(defmethod openlane_bag::LaneList
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lane_list __lane_list) ())
    ((:num_lanes __num_lanes) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _lane_list __lane_list)
   (setq _num_lanes (round __num_lanes))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lane_list
   (&rest __lane_list)
   (if (keywordp (car __lane_list))
       (send* _lane_list __lane_list)
     (progn
       (if __lane_list (setq _lane_list (car __lane_list)))
       _lane_list)))
  (:num_lanes
   (&optional __num_lanes)
   (if __num_lanes (setq _num_lanes __num_lanes)) _num_lanes)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; openlane_bag/Lane[] _lane_list
    (apply #'+ (send-all _lane_list :serialization-length)) 4
    ;; int32 _num_lanes
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; openlane_bag/Lane[] _lane_list
     (write-long (length _lane_list) s)
     (dolist (elem _lane_list)
       (send elem :serialize s)
       )
     ;; int32 _num_lanes
       (write-long _num_lanes s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; openlane_bag/Lane[] _lane_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lane_list (let (r) (dotimes (i n) (push (instance openlane_bag::Lane :init) r)) r))
     (dolist (elem- _lane_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32 _num_lanes
     (setq _num_lanes (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get openlane_bag::LaneList :md5sum-) "7c734ba2a0461f54efa410edc16ce01f")
(setf (get openlane_bag::LaneList :datatype-) "openlane_bag/LaneList")
(setf (get openlane_bag::LaneList :definition-)
      "Header header
Lane[] lane_list
int32 num_lanes
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: openlane_bag/Lane
LanePoint[] lane
int32 num_points
int32 attribute  # attribute: the left-left lane is 1, the left lane is 2, the right lane is 3, and the right-right lane is 4.
int32 track_id
int32 category

================================================================================
MSG: openlane_bag/LanePoint
float32 x
float32 y
float32 z
float32 visibility
")



(provide :openlane_bag/LaneList "7c734ba2a0461f54efa410edc16ce01f")


