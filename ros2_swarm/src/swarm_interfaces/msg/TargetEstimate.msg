# One agent's target-centre estimate, broadcast on the flat /target_estimate topic so blind
# agents in SEARCH can point and drift toward it. Never used for readiness (own perception only).

std_msgs/Header header   # stamp = estimate time; frame_id "swarm_map"
string robot             # sender namespace (e.g. RM3); receivers drop their own
float64 x                # map-frame centroid of the sender's landmark memory [m]
float64 y
uint16 count             # landmarks behind the estimate
