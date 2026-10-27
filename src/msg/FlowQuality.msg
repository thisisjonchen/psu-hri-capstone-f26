# Optical-flow reliability for one camera frame, for logging only; not used by control.
std_msgs/Header header
# False when the frame was rejected (too few blocks see nearby floor).
bool valid
# Motion-vector blocks behind the velocity; 0 when rejected or when motion is below one pixel.
uint16 blocks
