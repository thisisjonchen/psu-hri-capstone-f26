# Unfiltered flight-controller samples from one FC loop, for logging only; not used by control.
# Published only while subscribed. Values are exactly as received over MSP.
std_msgs/Header header
# MSP RAW_IMU accelerometer and gyro in the FC's IMU frame (x forward, y left, z up).
# Accelerometer: 2048 counts per g (params/multiwii.yaml). Gyro: firmware units (deg/s on Betaflight).
int16[3] accel
int16[3] gyro
# MSP MOTOR outputs for motors 1-4, as commanded to the ESCs.
uint16[4] motors
