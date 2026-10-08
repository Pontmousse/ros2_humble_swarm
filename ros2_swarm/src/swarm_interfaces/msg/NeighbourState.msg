# One neighbour as received over the UDP state broadcast.
# valid is false once the entry goes stale; the pose is then the last known one.

uint8 robot_id
float64 x
float64 y
float64 yaw
float64 vx
float64 vy
float64 omega
float64 age
bool valid
