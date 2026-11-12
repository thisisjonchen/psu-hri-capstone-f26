# PALS MSP interval. header.stamp maps the FC interval end into the ROS clock.
# FC timing is measured in sensor tasks, not at hardware acquisition.
# Rotation is xyzw, end body axes relative to interval start (fc_msp axes).
# Delta velocity is specific force integrated in the starting body axes (m/s),
# including gravity. Firmware uses the existing 9.8 m/s^2 per g convention.
std_msgs/Header header
uint32 sequence
uint32 sample_time_us
uint32 integration_time_us
bool valid
float32[4] delta_rotation
float32[3] delta_velocity
