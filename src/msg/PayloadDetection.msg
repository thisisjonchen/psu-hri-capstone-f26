# Latest yellow-payload observation; all coordinates and areas are in pixels.
# header is copied from the source image, including for cached/offline results.
# This is a color candidate, not a unique payload identity or a metric pose.
std_msgs/Header header
bool camera_online
bool detected
uint32 image_width
uint32 image_height
float32 center_x
float32 center_y
uint32 bbox_x
uint32 bbox_y
uint32 bbox_width
uint32 bbox_height
float32 area
