# go2_msgs/msg/SafetyAlert.msg
# Published by safety_monitor_node when a hazard is detected.
std_msgs/Header header
string alert_type      # EMERGENCY_STOP | SLOWDOWN | STAIRS_DETECTED | DROP_DETECTED | NARROW_PASSAGE
float32 distance       # Relevant distance measurement (meters)
string description     # Human-readable description
