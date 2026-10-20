# Peer telemetry only: no route, target, flight mode, or heartbeat commands.
# header.frame_id is the common map; stamp is the oldest source measurement.
std_msgs/Header header
string drone_id
string recipient_id
uint32 recipient_generation
geometry_msgs/PoseWithCovariance pose
# XY velocity expressed in the same map frame.
geometry_msgs/Vector3 velocity
