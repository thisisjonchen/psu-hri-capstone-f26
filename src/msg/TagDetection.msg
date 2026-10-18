# Results from the latest camera frame. Empty arrays mean no tags are visible.
std_msgs/Header header
bool camera_online
int32[] tag_ids
string[] zones
# Pixel center and image dimensions from the same, unrectified frame as header.
# Empty arrays mean no detected tags; arrays match tag_ids in order.
float32[] center_x
float32[] center_y
uint32 image_width
uint32 image_height
