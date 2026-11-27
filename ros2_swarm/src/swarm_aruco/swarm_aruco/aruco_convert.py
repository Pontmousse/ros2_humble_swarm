"""v2 drivers container's ArucoMarkerArray -> our Markers (used by swarm_target's perception node).

aruco/landmarks -> landmarks, aruco/markers -> targets, in aruco_detector's output contract:
body frame in cm (x = z_cam * sx forward, y = -x_cam * sy left), normals [right, forward] from the
marker +Z axis projected on the camera x-z plane, ids in the same order. The header (V4L2 capture
stamp) is copied, and empty frames are forwarded, so a live camera that sees nothing stays fresh.
"""
import math

from swarm_interfaces.msg import CoordXY, Markers


def convert(markers, scale, max_range_cm):
    """ArucoMarker list -> (positions, normals, ids). Drops uncalibrated (NaN) poses and poses beyond max_range_cm (0 = off)."""
    pos, nrm, ids = [], [], []
    for m in markers:
        p, d = m.pose.position, m.direction
        x, y = 100.0 * p.z * scale[0], -100.0 * p.x * scale[1]
        if not (math.isfinite(x) and math.isfinite(y)) or (max_range_cm > 0 and math.hypot(x, y) > max_range_cm): continue
        dx, dz = (-d.x, -d.z) if d.z < 0 else (d.x, d.z)
        n = math.hypot(dx, dz)
        pos.append(CoordXY(x=x, y=y)); nrm.append(CoordXY(x=dx / n, y=dz / n) if n > 1e-6 else CoordXY(x=dx, y=dz)); ids.append(int(m.id))
    return pos, nrm, ids


def to_markers(msg, scale, max_cm):
    """ArucoMarkerArray -> Markers with the same header (capture stamp); empty frames stay empty, not dropped."""
    pos, nrm, ids = convert(msg.markers, scale, max_cm)
    return Markers(header=msg.header, markers=pos, markers_normals=nrm, ids=ids)
