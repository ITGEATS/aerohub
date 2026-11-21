std_msgs/Header header
builtin_interfaces/Time observation_stamp
builtin_interfaces/Time valid_until
string session_id
uint64 generation
uint64 capture_id
string status
bool valid
# True after the confirmed track has valid velocity evidence.
bool velocity_valid
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64[36] covariance
string reason
