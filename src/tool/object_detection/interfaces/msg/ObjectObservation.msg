# Whether 3D positioning was requested (even if no objects were detected).
bool position_requested
# 2D: optical frame / RGB stamp. 3D: output frame / depth stamp when available.
std_msgs/Header header
std_msgs/Header image_header
std_msgs/Header depth_header
string model_version
uav_object_detection_interfaces/ObjectDetection[] detections
# Original user text and deployed backend interpretation; labels remain model phrases.
string original_query
string normalized_query
string detector_backend
string request_id
uint64 generation
uint64 tool_generation
uint64 worker_generation
