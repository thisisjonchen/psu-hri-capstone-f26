# Each drone's handshake state, re-published continuously.
# state: idle, proposed, accepted, rejected, executing, cancelled, done, failed or finished.
std_msgs/Header header
string drone_id
uint32 plan_id
string state
string reason
