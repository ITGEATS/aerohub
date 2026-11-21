# Original image pixel coordinates; clipping is specified by the model service contract.
float64[4] xyxy
string category
float64 confidence
# Optional binary instance mask in original image geometry, mono8 values 0/1.
# Empty width/height means a Detect model returned no mask. Header matches source.
sensor_msgs/Image mask
# 2D association only; zero means no ID. Scoped by DetectionFrame.tracking_generation.
uint64 track_id
