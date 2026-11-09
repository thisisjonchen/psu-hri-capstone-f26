# Leader's proposed coordinated action; the follower executes it only after accepting.
std_msgs/Header header
uint32 plan_id
string action
float64[] params
# Wall-clock start shared by both drones (NTP-synced clocks).
time start_time
# After the action ends, hold X/Y this long; land if no next plan has started by then.
float64 hold_s
