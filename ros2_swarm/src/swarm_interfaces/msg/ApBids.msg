# One agent's auction preference vector, broadcast on the flat /ap_bids topic.

std_msgs/Header header   # stamp = when bids computed; frame_id "swarm_map"
uint8 robot_id           # bidder id = auction tiebreak key
int32[] ap_ids           # global AP ids currently reachable (from targets msg.ids)
float64[] bids           # parallel to ap_ids; higher = stronger; non-finite rejected
int32 claimed_ap_id      # AP this agent currently claims (-1 = none)
bool locked              # committed to claimed_ap_id
