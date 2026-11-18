# External trajectory in a fixed, vehicle-local ENU frame. Never use a moving TF.
uint32 interface_version
string task_id
string vehicle_id
uint64 revision
string frame_id
builtin_interfaces/Time generated_at
builtin_interfaces/Time valid_from
bool is_final
# VELOCITY=1, ACCELERATION=2, YAW=4 (same as VehicleTrajectory).
uint8 valid_fields
uav_planner_interfaces/TimedTrajectoryPoint[] points
float64 max_speed

# Optional executor-local target yaw (local ENU); mutually exclusive with YAW fields.
bool continuous_yaw
float64 target_yaw
float64 max_yaw_rate
uint64 yaw_revision
builtin_interfaces/Time yaw_valid_until
