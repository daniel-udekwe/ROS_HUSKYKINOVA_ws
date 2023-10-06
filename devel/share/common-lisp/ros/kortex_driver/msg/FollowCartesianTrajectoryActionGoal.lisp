; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude FollowCartesianTrajectoryActionGoal.msg.html

(cl:defclass <FollowCartesianTrajectoryActionGoal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal_id
    :reader goal_id
    :initarg :goal_id
    :type actionlib_msgs-msg:GoalID
    :initform (cl:make-instance 'actionlib_msgs-msg:GoalID))
   (goal
    :reader goal
    :initarg :goal
    :type kortex_driver-msg:FollowCartesianTrajectoryGoal
    :initform (cl:make-instance 'kortex_driver-msg:FollowCartesianTrajectoryGoal)))
)

(cl:defclass FollowCartesianTrajectoryActionGoal (<FollowCartesianTrajectoryActionGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FollowCartesianTrajectoryActionGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FollowCartesianTrajectoryActionGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<FollowCartesianTrajectoryActionGoal> is deprecated: use kortex_driver-msg:FollowCartesianTrajectoryActionGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FollowCartesianTrajectoryActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:header-val is deprecated.  Use kortex_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal_id-val :lambda-list '(m))
(cl:defmethod goal_id-val ((m <FollowCartesianTrajectoryActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:goal_id-val is deprecated.  Use kortex_driver-msg:goal_id instead.")
  (goal_id m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <FollowCartesianTrajectoryActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:goal-val is deprecated.  Use kortex_driver-msg:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FollowCartesianTrajectoryActionGoal>) ostream)
  "Serializes a message object of type '<FollowCartesianTrajectoryActionGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FollowCartesianTrajectoryActionGoal>) istream)
  "Deserializes a message object of type '<FollowCartesianTrajectoryActionGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FollowCartesianTrajectoryActionGoal>)))
  "Returns string type for a message object of type '<FollowCartesianTrajectoryActionGoal>"
  "kortex_driver/FollowCartesianTrajectoryActionGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FollowCartesianTrajectoryActionGoal)))
  "Returns string type for a message object of type 'FollowCartesianTrajectoryActionGoal"
  "kortex_driver/FollowCartesianTrajectoryActionGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FollowCartesianTrajectoryActionGoal>)))
  "Returns md5sum for a message object of type '<FollowCartesianTrajectoryActionGoal>"
  "60bf486344cb17b10eba10bac039409d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FollowCartesianTrajectoryActionGoal)))
  "Returns md5sum for a message object of type 'FollowCartesianTrajectoryActionGoal"
  "60bf486344cb17b10eba10bac039409d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FollowCartesianTrajectoryActionGoal>)))
  "Returns full string definition for message of type '<FollowCartesianTrajectoryActionGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%FollowCartesianTrajectoryGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: kortex_driver/FollowCartesianTrajectoryGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#The trajectory to follow~%CartesianWaypoint[] trajectory~%duration goal_time_tolerance~%bool use_optimal_blending~%~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FollowCartesianTrajectoryActionGoal)))
  "Returns full string definition for message of type 'FollowCartesianTrajectoryActionGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%FollowCartesianTrajectoryGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: kortex_driver/FollowCartesianTrajectoryGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#The trajectory to follow~%CartesianWaypoint[] trajectory~%duration goal_time_tolerance~%bool use_optimal_blending~%~%================================================================================~%MSG: kortex_driver/CartesianWaypoint~%~%Pose pose~%uint32 reference_frame~%float32 maximum_linear_velocity~%float32 maximum_angular_velocity~%float32 blending_radius~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FollowCartesianTrajectoryActionGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FollowCartesianTrajectoryActionGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'FollowCartesianTrajectoryActionGoal
    (cl:cons ':header (header msg))
    (cl:cons ':goal_id (goal_id msg))
    (cl:cons ':goal (goal msg))
))
