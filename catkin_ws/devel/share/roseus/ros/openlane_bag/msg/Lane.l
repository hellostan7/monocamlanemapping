;; Auto-generated. Do not edit!


(when (boundp 'openlane_bag::Lane)
  (if (not (find-package "OPENLANE_BAG"))
    (make-package "OPENLANE_BAG"))
  (shadow 'Lane (find-package "OPENLANE_BAG")))
(unless (find-package "OPENLANE_BAG::LANE")
  (make-package "OPENLANE_BAG::LANE"))

(in-package "ROS")
;;//! \htmlinclude Lane.msg.html


(defclass openlane_bag::Lane
  :super ros::object
  :slots (_lane _num_points _attribute _track_id _category ))

(defmethod openlane_bag::Lane
  (:init
   (&key
    ((:lane __lane) ())
    ((:num_points __num_points) 0)
    ((:attribute __attribute) 0)
    ((:track_id __track_id) 0)
    ((:category __category) 0)
    )
   (send-super :init)
   (setq _lane __lane)
   (setq _num_points (round __num_points))
   (setq _attribute (round __attribute))
   (setq _track_id (round __track_id))
   (setq _category (round __category))
   self)
  (:lane
   (&rest __lane)
   (if (keywordp (car __lane))
       (send* _lane __lane)
     (progn
       (if __lane (setq _lane (car __lane)))
       _lane)))
  (:num_points
   (&optional __num_points)
   (if __num_points (setq _num_points __num_points)) _num_points)
  (:attribute
   (&optional __attribute)
   (if __attribute (setq _attribute __attribute)) _attribute)
  (:track_id
   (&optional __track_id)
   (if __track_id (setq _track_id __track_id)) _track_id)
  (:category
   (&optional __category)
   (if __category (setq _category __category)) _category)
  (:serialization-length
   ()
   (+
    ;; openlane_bag/LanePoint[] _lane
    (apply #'+ (send-all _lane :serialization-length)) 4
    ;; int32 _num_points
    4
    ;; int32 _attribute
    4
    ;; int32 _track_id
    4
    ;; int32 _category
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; openlane_bag/LanePoint[] _lane
     (write-long (length _lane) s)
     (dolist (elem _lane)
       (send elem :serialize s)
       )
     ;; int32 _num_points
       (write-long _num_points s)
     ;; int32 _attribute
       (write-long _attribute s)
     ;; int32 _track_id
       (write-long _track_id s)
     ;; int32 _category
       (write-long _category s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; openlane_bag/LanePoint[] _lane
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lane (let (r) (dotimes (i n) (push (instance openlane_bag::LanePoint :init) r)) r))
     (dolist (elem- _lane)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32 _num_points
     (setq _num_points (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _attribute
     (setq _attribute (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _track_id
     (setq _track_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _category
     (setq _category (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get openlane_bag::Lane :md5sum-) "bc501fcdae255d97e8770b24dbfa3d94")
(setf (get openlane_bag::Lane :datatype-) "openlane_bag/Lane")
(setf (get openlane_bag::Lane :definition-)
      "LanePoint[] lane
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



(provide :openlane_bag/Lane "bc501fcdae255d97e8770b24dbfa3d94")


