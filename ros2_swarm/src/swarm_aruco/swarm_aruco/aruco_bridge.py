"""Driver-stack bridge: the v2 drivers container's ArucoMarkerArray -> our Markers.

aruco/landmarks -> landmarks_unf, aruco/markers -> targets_unf, in aruco_detector's output contract:
body frame in cm (x = z_cam * sx forward, y = -x_cam * sy left), normals [right, forward] from the
marker +Z axis projected on the camera x-z plane, ids in the same order. The header (V4L2 capture
stamp) is copied, and empty frames are forwarded, so a live camera that sees nothing stays fresh.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from swarm_interfaces.msg import ArucoMarkerArray, CoordXY, Markers


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


class ArucoBridge(Node):
    def __init__(self):
        super().__init__('aruco_bridge')
        qos = QoSProfile(depth=int(self.declare_parameter('qos_depth', 10).value), reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        for name, src, dst in (('landmark', 'aruco/landmarks', 'landmarks_unf'), ('target', 'aruco/markers', 'targets_unf')):
            scale = tuple(self.declare_parameter(f'{name}_scale', [1.0, 1.0]).value)
            max_cm = 100.0 * float(self.declare_parameter(f'{name}_max_range', 0.0).value)  # m, like aruco_detector
            pub = self.create_publisher(Markers, dst, qos)
            self.create_subscription(ArucoMarkerArray, src, lambda msg, pub=pub, scale=scale, max_cm=max_cm: pub.publish(self.to_markers(msg, scale, max_cm)), qos)
            self.get_logger().info(f'{src} -> {dst}: scale {scale}, max_range {max_cm / 100.0} m')

    @staticmethod
    def to_markers(msg, scale, max_cm):
        pos, nrm, ids = convert(msg.markers, scale, max_cm)
        return Markers(header=msg.header, markers=pos, markers_normals=nrm, ids=ids)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoBridge()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
