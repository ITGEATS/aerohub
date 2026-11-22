# Capture timestamp and fixed world frame of the original 3D measurement.
std_msgs/Header header
string session_id
uint64 generation
uint64 capture_id
# Increments each time the target is confirmed after acquisition or loss.
uint64 track_epoch
geometry_msgs/Point position
