# One agent's synchronized-capture readiness, broadcast on the flat /readiness topic.

std_msgs/Header header   # stamp = snapshot time; frame_id "swarm_map"
uint8 robot_id
bool ready               # locked AND claimed-AP in range AND raw perception live
uint8 degree             # in-range valid-neighbour count (retained for monitoring)
float64 action_time      # proposed absolute capture time (sec, ROS clock); 0 = not proposing
uint64 round_id         # negotiation generation; older rounds cannot contribute
bool armed              # action_time is frozen for this round
