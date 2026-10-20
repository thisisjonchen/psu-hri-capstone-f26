# A fresh, unrectified camera observation; never republished from a cache.
std_msgs/Header header
int32 tag_id
uint32 image_width
uint32 image_height
# Canonical printed-tag order: top-left, top-right, bottom-right, bottom-left.
# Top refers to the standard AprilRobotics tag image, not the camera image.
float32[4] corner_x
float32[4] corner_y
