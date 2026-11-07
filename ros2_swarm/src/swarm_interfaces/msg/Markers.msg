# Array of markers

# Stamp marks when the detection was VALID (set by the detector when it last
# ran), not when the message was published. The detector republishes its last
# frame at ~100 Hz, so a frozen camera keeps publishing at full rate; consumers
# must compare this stamp's age to tell a live stream from a frozen one.
std_msgs/Header header
CoordXY[] markers
CoordXY[] markers_normals

# ArUco marker IDs, parallel to markers[] / markers_normals[] (same length and
# order). Carries the fiducial identity through the pipeline so a landmark
# estimator can do ID-based data association. Empty when the source has no
# identity (older detectors, or a producer that does not fill it) -- consumers
# that ignore the field keep working unchanged.
int32[] ids
