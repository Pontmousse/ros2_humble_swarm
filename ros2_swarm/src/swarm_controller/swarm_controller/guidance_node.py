"""guidance: the wrench contributors flocking + avoidance, pointing and capture, in one process.

Replaces motion_planner, pointing_planner and cp_controller (nodes-to-functions Phase 3). Each input has one
subscription, fanned out to the laws that read it; one tick publishes every wrench. This file only wires
parameters, topics, services and timers; the laws live in flocking, pointing, capture_law and shared_target.

Velocity sources stay distinct: flocking and pointing damp on the PHYSICAL localization/odom, capture servos on
the VIRTUAL craft (virtual_spacecraft/odom).
"""
import rclpy
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import SetBool
from swarm_interfaces.msg import Markers, NeighbourStates, TargetEstimate
from swarm_interfaces.msg import StatePos as State

from .capture_law import CaptureLaw
from .flocking import Flocking
from .pointing import Pointing
from .shared_target import SHARE_QOS, now_s
from .zero_gate import ZeroGatedPublisher


class GuidanceNode(Node):
    def __init__(self):
        super().__init__('guidance')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        init = float(p("init_period", 1.0))
        period = float(p("timer_frequency", 0.05))
        t_scale, r_scale = float(p("translation_response_scale", 1.0)), float(p("rotation_response_scale", 1.0))
        k, fresh = int(p("landmark_nearest_k", 5)), float(p("landmarks_freshness_timeout", 0.3))
        share, share_timeout, share_rate = bool(p("share_target", True)), float(p("share_timeout", 2.0)), float(p("share_rate", 2.0))
        depth = int(p("qos_depth", 1))
        reliable = QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        mode_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        gated = lambda topic, qos: ZeroGatedPublisher(self.create_publisher(Wrench, topic, qos))  # noqa: E731
        law_p = lambda prefix: (lambda name, default: p(f"{prefix}.{name}", default))  # noqa: E731

        self.flocking = f = Flocking(self, gated("avoidance_wrench", 10), gated("flocking_wrench", 10), init, t_scale, k, fresh)
        self.pointing = pt = Pointing(self, law_p("pointing"), gated("pointing_wrench", reliable),
                                      self.create_publisher(TargetEstimate, '/target_estimate', SHARE_QOS) if share else None,
                                      init, r_scale, k, fresh, share_timeout)
        direct = bool(p("capture.direct_cmd_vel", False))
        self.capture = c = CaptureLaw(self, law_p("capture"), None if direct else gated("capture_wrench", reliable),
                                      gated("preparation_wrench", reliable),
                                      self.create_publisher(Twist, 'cmd_vel', reliable) if direct else None,
                                      init, t_scale, r_scale)

        fan = lambda *cbs: (lambda m: [cb(m) for cb in cbs])  # noqa: E731
        self.create_subscription(State, 'pose', fan(f.pose_callback, pt.pose_callback, c.pose_callback), reliable)
        self.create_subscription(Markers, 'landmarks', fan(f.landmarks_callback, pt.landmarks_callback), reliable)
        self.create_subscription(Odometry, 'localization/odom', fan(f.odom_callback, pt.odom_callback), reliable)
        self.create_subscription(String, 'current_mode', fan(pt.mode_callback, c.mode_callback), mode_qos)
        self.create_subscription(NeighbourStates, 'comms/neighbours', f.neighbours_callback, latest)
        self.create_subscription(Markers, 'targets', c.target_callback, reliable)
        self.create_subscription(Int32, 'assigned_ap', c.assigned_ap_callback, reliable)
        self.create_subscription(Bool, 'assignment_locked', c.assignment_locked_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, 'virtual_spacecraft/odom', c.spacecraft_callback, latest)
        if direct:
            self.create_subscription(Twist, 'cmd_vel_nav', c.cmd_vel_nav_callback, reliable)
        if share:
            self.create_subscription(TargetEstimate, '/target_estimate', lambda m: pt.shared.add(m, now_s(self)), SHARE_QOS)
            self.create_timer(1.0 / share_rate, pt.publish_estimate)

        # Mode gates, called by mode_supervisor; same service names as the old nodes.
        self.create_service(SetBool, 'encapsulation/enable', f.set_encapsulation)
        self.create_service(SetBool, 'toggle_controller', c.handle_controller)
        self.create_service(SetBool, 'toggle_dock', c.handle_dock)
        self.create_timer(period, self.tick)
        self.get_logger().info(f"guidance: {1.0 / period:.0f} Hz, direct_cmd_vel={direct}, share_target={share}")

    def tick(self):
        self.flocking.publish_target_position()
        self.pointing.publish_target_orientation()
        self.capture.publish_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
