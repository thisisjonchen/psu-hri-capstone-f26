# Heights and velocities are room-frame Z (up), in m and m/s; biases are m/s^2.
# Header stamps the current IMU output. Cached range fields retain their own stamp.
std_msgs/Header header
time fusion_stamp
time range_stamp
# Deduplicate cached updates by sequence; gaps identify updates missed between snapshots.
uint64 range_sequence
bool range_accepted
bool range_ground_reset
# Innovation is measured minus predicted slant range; NaN for invalid geometry.
float64 range_innovation
float64 range_height_before
float64 range_height_after
float64 range_velocity_before
float64 range_velocity_after
float64 range_bias_up_before
float64 range_bias_up_after
float64 fusion_height
float64 fusion_velocity
float64 accel_bias_up
float64 projected_height
float64 projected_velocity
float64 output_height
float64 output_velocity
