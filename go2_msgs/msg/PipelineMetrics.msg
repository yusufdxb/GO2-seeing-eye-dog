std_msgs/Header header
string node_name          # "perception" or "safety_monitor"
uint64 frame_id           # monotonic frame counter
# perception stages (0.0 when not applicable, e.g. on safety node)
float64 cv_bridge_rgb_ms
float64 yolo_inference_ms
float64 depth_sampling_ms
float64 backproject_ms
float64 visualization_ms
float64 publish_ms
# safety stages (0.0 on perception node)
float64 safety_obstacle_ms
float64 safety_stair_ms
float64 safety_drop_ms
float64 safety_passage_ms
float64 safety_total_ms
# common
float64 total_ms          # callback entry to exit
