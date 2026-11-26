# One detected ArUco marker, in the camera optical frame (x right, y down, z forward).
# Pose fields are NaN when the camera has no calibration.
int32 id
geometry_msgs/Pose pose          # position (m) + orientation (quaternion)
float64 distance                 # |position|, m
geometry_msgs/Vector3 direction  # marker facing direction: its +Z axis (unit vector)
