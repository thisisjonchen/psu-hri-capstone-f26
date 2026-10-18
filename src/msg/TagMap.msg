# Observed AprilTag centers in the UWB map frame. DISPLAY/MAPPING ONLY;
# these estimates are not fed back into flight control or localization.
# header.stamp = when this persistent map snapshot was updated, not sensor time.
std_msgs/Header header
int32[] tag_ids
string[] zones
geometry_msgs/Point[] positions
uint32[] sample_counts
