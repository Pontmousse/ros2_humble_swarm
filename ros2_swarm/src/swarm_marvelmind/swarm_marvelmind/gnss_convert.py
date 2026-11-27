"""v2 drivers container's gnss/position (PointStamped, m) -> HedgePositionAddressed (pose_kf's Marvelmind input).

timestamp_ms comes from the header stamp, so pose_kf's (x, y, timestamp_ms) fresh gate sees every fix as new.
"""
from marvelmind_ros2_msgs.msg import HedgePositionAddressed


def to_hedge(msg, beacon):
    s = msg.header.stamp
    return HedgePositionAddressed(address=beacon, timestamp_ms=s.sec * 1000 + s.nanosec // 1_000_000,
                                  x_m=msg.point.x, y_m=msg.point.y, z_m=msg.point.z, flags=0)
