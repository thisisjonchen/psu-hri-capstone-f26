# Header: latest projected IMU time, map frame; positive height/velocity is up.
std_msgs/Header header
# Delayed EKF horizon. EKF values below are projected to header.stamp.
time fusion_stamp
float64 height_ekf
float64 height_output
float64 velocity_ekf
float64 velocity_output
# Learned accelerometer bias (m/s^2): body FLU Z and its map-up projection.
float64 accel_bias_body_z
float64 accel_bias_up
# Latest ToF correction attempt at the delayed horizon, repeated until updated.
# Sequence 0 means no attempt; a sequence gap means intermediate attempts were missed.
uint64 range_sequence
time range_stamp
# Measured minus predicted slant range (m); NaN if beam geometry prevented evaluation.
float64 range_innovation
bool range_accepted
# A disarmed ground-height reset is separate from an accepted Kalman correction.
bool range_ground_reset
