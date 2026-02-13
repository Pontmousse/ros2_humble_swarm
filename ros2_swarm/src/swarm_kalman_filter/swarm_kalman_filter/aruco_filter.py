import math
import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import CoordXY, Markers
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ArucoFilterNode(Node):
    def __init__(self):
        super().__init__('ArucoFilter')

        # # # # # # # # # # # # # # # # # # # # # # # # # # # 
        # For QoS
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'RELIABLE')
        self.declare_parameter('qos_history', 'KEEP_LAST')

        qos_depth = self.get_parameter('qos_depth').value
        qos_reliability = self.get_parameter('qos_reliability').value
        qos_history = self.get_parameter('qos_history').value

        # Validate QoS settings       
        valid_reliabilities = ['RELIABLE', 'BEST_EFFORT']
        valid_histories = ['KEEP_LAST', 'KEEP_ALL']

        if qos_reliability not in valid_reliabilities:
            raise ValueError(f"Invalid qos_reliability: {qos_reliability}")
        if qos_history not in valid_histories:
            raise ValueError(f"Invalid qos_history: {qos_history}")

        # QoS Settings 
        self.qos_profile = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE if qos_reliability == 'RELIABLE' else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST if qos_history == 'KEEP_LAST' else HistoryPolicy.KEEP_ALL
        )
        # # # # # # # # # # # # # # # # # # # # # # # # # # # 

        # α controls smoothing: 1.0=no filter, 0.0=freeze first value
        self.declare_parameter('alpha', 0.3)
        self.alpha = float(self.get_parameter('alpha').value)

        # State: last filtered centroid & normal
        self._filtered_pos: CoordXY | None = None
        self._filtered_norm: CoordXY | None = None

        # Sub + Pub
        self.sub = self.create_subscription(
            Markers, 'markers_unf', self.cb_markers, self.qos_profile)
        self.pub = self.create_publisher(
            Markers, 'markers', self.qos_profile)

        self.get_logger().info(f'ArucoFilter started (alpha={self.alpha:.3f})')

    def cb_markers(self, msg: Markers):
        # If no detections, just re-publish previous filtered constellation
        if not msg.markers:
            if self._filtered_pos is None:
                return
            out = Markers()
            # replicate last filtered constellation size (empty here)
            out.markers = []
            out.markers_normals = []
            self.pub.publish(out)
            return

        # 1) Compute raw centroids
        n = len(msg.markers)
        sum_x = sum(m.x for m in msg.markers)
        sum_y = sum(m.y for m in msg.markers)
        avg_pos = CoordXY(x=sum_x/n, y=sum_y/n)

        sum_nx = sum(nm.x for nm in msg.markers_normals)
        sum_ny = sum(nm.y for nm in msg.markers_normals)
        avg_norm = CoordXY(x=sum_nx/n, y=sum_ny/n)

        # 2) Update filtered centroids
        if self._filtered_pos is None:
            # init on first message
            self._filtered_pos = CoordXY(x=avg_pos.x, y=avg_pos.y)
            self._filtered_norm = CoordXY(x=avg_norm.x, y=avg_norm.y)
        else:
            α = self.alpha
            fp = self._filtered_pos
            fn = self._filtered_norm

            fp.x = α * avg_pos.x + (1 - α) * fp.x
            fp.y = α * avg_pos.y + (1 - α) * fp.y
            fn.x = α * avg_norm.x + (1 - α) * fn.x
            fn.y = α * avg_norm.y + (1 - α) * fn.y

        # 3) Build output: shift each raw marker by (filtered_centroid - raw_centroid)
        out = Markers()
        out.markers = []
        out.markers_normals = []

        dx = self._filtered_pos.x - avg_pos.x
        dy = self._filtered_pos.y - avg_pos.y
        dnx = self._filtered_norm.x - avg_norm.x
        dny = self._filtered_norm.y - avg_norm.y

        for raw_m, raw_n in zip(msg.markers, msg.markers_normals):
            # position
            m = CoordXY()
            m.x = raw_m.x + dx
            m.y = raw_m.y + dy

            # normal: shift and renormalize
            nx = raw_n.x + dnx
            ny = raw_n.y + dny
            norm = math.hypot(nx, ny)
            nmsg = CoordXY()
            if norm > 1e-6:
                nmsg.x = nx / norm
                nmsg.y = ny / norm
            else:
                # fallback to filtered normal
                nmsg.x = self._filtered_norm.x
                nmsg.y = self._filtered_norm.y

            out.markers.append(m)
            out.markers_normals.append(nmsg)

        self.pub.publish(out)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

