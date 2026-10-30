# Firmware-processed MSP counts before host calibration or gravity removal.
# frame_id is fc_msp; conversion to body axes and SI units requires calibration.
# The timestamp marks packet receipt on the host, not sensor acquisition.
std_msgs/Header header
int16[3] accelerometer
int16[3] gyroscope
