"""gnss_bridge.to_hedge: v2 gnss/position -> mm_pos_unf (pure function, no node)."""
from geometry_msgs.msg import PointStamped

from swarm_marvelmind.gnss_bridge import to_hedge


def fix(sec, nsec, x, y, z):
    m = PointStamped()
    m.header.stamp.sec, m.header.stamp.nanosec = sec, nsec
    m.point.x, m.point.y, m.point.z = x, y, z
    return m


def test_fields_and_stamp():
    h = to_hedge(fix(1764100000, 987_654_321, 1.25, -0.5, 0.3), 6)
    assert (h.address, h.x_m, h.y_m, h.z_m, h.flags) == (6, 1.25, -0.5, 0.3, 0)
    assert h.timestamp_ms == 1764100000 * 1000 + 987


def test_successive_fixes_pass_pose_kf_fresh_gate():
    # pose_kf drops a fix whose (x_m, y_m, timestamp_ms) repeats; a stationary robot must still update.
    a, b = to_hedge(fix(10, 0, 1.0, 1.0, 0.0), 1), to_hedge(fix(10, 169_000_000, 1.0, 1.0, 0.0), 1)
    assert (a.x_m, a.y_m, a.timestamp_ms) != (b.x_m, b.y_m, b.timestamp_ms)
