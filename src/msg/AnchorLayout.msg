# Latest anchors reported by this drone's active PANS tag. Coordinates are meters.
# IDs are normalized four-digit hexadecimal UWB short addresses, not AN slot names.
# Arrays correspond by index. A partial/empty report must not resize an existing map.
std_msgs/Header header
string[] anchor_ids
geometry_msgs/Point[] positions
