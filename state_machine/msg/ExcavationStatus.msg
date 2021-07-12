#state_machine/ExcavationStatus 
std_msgs/UInt8 excavator_id
std_msgs/Int8 state
std_msgs/Bool bucket_full
std_msgs/Bool found_parking_site
geometry_msgs/Pose parking_pose
std_msgs/Int8 parking_side
std_msgs/Bool found_volatile
std_msgs/Bool found_hauler
std_msgs/Int8 counter
std_msgs/Float64 progress