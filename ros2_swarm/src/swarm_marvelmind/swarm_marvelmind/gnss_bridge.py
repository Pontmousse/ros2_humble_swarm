"""Driver-stack bridge: the v2 drivers container's gnss/position (PointStamped, m) -> our mm_pos_unf.

Replaces marvelmind_obs + hedgehog_obs when DRIVER_STACK=v2. Publishes once per fix (no timer
republish); timestamp_ms comes from the header stamp, so pose_kf's (x, y, timestamp_ms) fresh gate
sees every fix as new.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped
from marvelmind_ros2_msgs.msg import HedgePositionAddressed


def to_hedge(msg, beacon):
    s = msg.header.stamp
    return HedgePositionAddressed(address=beacon, timestamp_ms=s.sec * 1000 + s.nanosec // 1_000_000,
                                  x_m=msg.point.x, y_m=msg.point.y, z_m=msg.point.z, flags=0)


class GnssBridge(Node):
    def __init__(self):
        super().__init__('gnss_bridge')
        beacon = int(self.declare_parameter('beacon', 0).value)
        qos = QoSProfile(depth=int(self.declare_parameter('qos_depth', 10).value), reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        pub = self.create_publisher(HedgePositionAddressed, 'mm_pos_unf', qos)
        self.create_subscription(PointStamped, 'gnss/position', lambda m: pub.publish(to_hedge(m, beacon)), qos)
        self.get_logger().info(f'gnss/position -> mm_pos_unf (beacon {beacon})')


def main(args=None):
    rclpy.init(args=args)
    node = GnssBridge()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
