# go2_msgs/msg/ConfirmedTarget.msg
# Published when intent grounding has locked onto a confirmed caller.
std_msgs/Header header
geometry_msgs/Pose pose       # 3D position in camera frame
float32 confidence            # Fused audio-visual confidence [0, 1]
string target_id              # Optional re-ID label
