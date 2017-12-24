;; Auto-generated. Do not edit!


(when (boundp 'drift_car::Action)
  (if (not (find-package "DRIFT_CAR"))
    (make-package "DRIFT_CAR"))
  (shadow 'Action (find-package "DRIFT_CAR")))
(unless (find-package "DRIFT_CAR::ACTION")
  (make-package "DRIFT_CAR::ACTION"))

(in-package "ROS")
;;//! \htmlinclude Action.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass drift_car::Action
  :super ros::object
  :slots (_header _steering_angle _throttle ))

(defmethod drift_car::Action
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:steering_angle __steering_angle) 0.0)
    ((:throttle __throttle) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _steering_angle (float __steering_angle))
   (setq _throttle (float __throttle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:steering_angle
   (&optional __steering_angle)
   (if __steering_angle (setq _steering_angle __steering_angle)) _steering_angle)
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _steering_angle
    8
    ;; float64 _throttle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _steering_angle
       (sys::poke _steering_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _throttle
       (sys::poke _throttle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _steering_angle
     (setq _steering_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _throttle
     (setq _throttle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get drift_car::Action :md5sum-) "c4d5434aed0e942aaba81c50637751a2")
(setf (get drift_car::Action :datatype-) "drift_car/Action")
(setf (get drift_car::Action :definition-)
      "Header header
float64 steering_angle
float64 throttle

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
# 0: no frame
# 1: global frame
string frame_id

")



(provide :drift_car/Action "c4d5434aed0e942aaba81c50637751a2")


