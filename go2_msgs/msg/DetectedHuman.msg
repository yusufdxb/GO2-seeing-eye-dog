# go2_msgs/msg/DetectedHuman.msg
# A single detected human with 3D pose and bounding box

geometry_msgs/Pose pose       # 3D position in camera frame
float32 confidence            # YOLOv8 detection confidence [0, 1]
int32 bbox_x1                 # Bounding box top-left x (pixels)
int32 bbox_y1                 # Bounding box top-left y (pixels)
int32 bbox_x2                 # Bounding box bottom-right x (pixels)
int32 bbox_y2                 # Bounding box bottom-right y (pixels)
string track_id               # Optional re-ID track identifier
