# Whether 3D positioning was requested (even if no objects were detected).
bool position_requested
# 2D: optical frame / RGB stamp. 3D: output frame / depth stamp when available.
std_msgs/Header header
std_msgs/Header image_header
std_msgs/Header depth_header
string model_version
uav_object_detection_interfaces/ObjectDetection[] detections
