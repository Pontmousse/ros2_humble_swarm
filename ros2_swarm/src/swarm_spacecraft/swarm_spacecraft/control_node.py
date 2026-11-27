"""control: virtual spacecraft + bounding-box search + PID+FF tracker + velocity smoother, in one process.

Replaces virtual_spacecraft, bounding_box_search, nav2_pidff, nav2 velocity_smoother and its lifecycle manager
(nodes-to-functions Phase 4). One timer runs the chain in order, so the tracker renders the reference integrated
in the same tick and cmd_vel goes out at the loop rate:

    box.update -> craft.update (publishes virtual_spacecraft/odom) -> tracker.update -> smoother.update (cmd_vel)

Each class keeps its own staleness rule (spacecraft freeze at virtual_spacecraft.odometry_timeout, tracker stop at
nav2_pidff.odometry_timeout, smoother velocity_timeout). The classes publish through fan-outs, which is how the box
wrench reaches the craft's bounding_box source and the reference reaches box and tracker, in-process. This file
only wires parameters, topics, services and the timer; the logic lives in virtual_craft, bounding_box_search,
tracking and smoother.
"""
from functools import partial

import rclpy
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger
from swarm_controller.bounding_box_search import BoxSearch
from swarm_controller.shared_target import SHARE_QOS, now_s
from swarm_controller.zero_gate import ZeroGatedPublisher
from swarm_interfaces.msg import TargetEstimate
from swarm_nav2_controller.smoother import VelocitySmoother
from swarm_nav2_controller.tracking import Tracker

from .virtual_craft import VirtualCraft

# Shared values from the yaml's top-level /** block, read unprefixed; everything else is <class prefix>.<name>.
GLOBAL = ('x_min', 'x_max', 'y_min', 'y_max', 'translation_response_scale', 'rotation_response_scale',
          'share_target', 'share_timeout')
INTERNAL_SOURCE = 'bounding_box'   # the box's wrench is fed in-process, not subscribed


class Fanout:
    def __init__(self, *sinks): self.sinks = list(sinks)
    def publish(self, msg): [sink(msg) for sink in self.sinks]


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        values = {}
        param = lambda n, d: values[n] if n in values else values.setdefault(n, self.declare_parameter(n, d).value)  # noqa: E731
        law_p = lambda prefix: (lambda n, d: param(n if n in GLOBAL else f'{prefix}.{n}', d))  # noqa: E731
        debug, direct = bool(param('debug_topics', True)), bool(param('direct_cmd_vel', False))
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        pub = lambda t, topic: self.create_publisher(t, topic, qos)  # noqa: E731

        ref = Fanout(pub(Odometry, 'virtual_spacecraft/odom').publish)
        self.craft = VirtualCraft(self, law_p('virtual_spacecraft'), ref, pub(Wrench, 'virtual_spacecraft/applied_wrench'))
        period = self.craft.timer_frequency
        # direct_cmd_vel: guidance (capture) owns cmd_vel and muxes this nav-chain output from cmd_vel_nav.
        self.smoother = VelocitySmoother(self, law_p('velocity_smoother'), pub(Twist, 'cmd_vel_nav' if direct else 'cmd_vel'),
                                         1.0 / period)
        self.tracker = Tracker(self, law_p('nav2_pidff'), Fanout(self.smoother.command_callback,
                               *([pub(Twist, 'cmd_vel_raw').publish] if debug else [])), period)
        self.box = BoxSearch(self, law_p('bounding_box_search'), Fanout(
            partial(self.craft.wrench_callback, INTERNAL_SOURCE),
            *([ZeroGatedPublisher(pub(Wrench, 'spacecraft_wrench')).publish] if debug else [])))
        ref.sinks += [self.box.reference_callback, self.tracker.reference_callback]

        self.create_subscription(Odometry, 'localization/odom',
                                 lambda m: (self.craft.odometry_callback(m), self.tracker.odometry_callback(m)), qos)
        for name, topic in self.craft.wrench_topics.items():
            if name != INTERNAL_SOURCE:
                self.create_subscription(Wrench, topic, partial(self.craft.wrench_callback, name), qos)
        if self.box.share_target:
            self.create_subscription(TargetEstimate, '/target_estimate', lambda m: self.box.shared.add(m, now_s(self)),
                                     SHARE_QOS)
        self.create_service(Trigger, 'virtual_spacecraft/reset', self.craft.reset_callback)
        self.create_service(Trigger, 'virtual_spacecraft/arm', self.craft.arm_callback)
        self.create_service(SetBool, 'search/enable', self.box.handle_enable)
        self.stop_publisher = self.smoother.publisher
        self.create_timer(period, self.tick)
        self.get_logger().info(f'control: {1.0 / period:.0f} Hz, direct_cmd_vel={direct}, debug_topics={debug}')

    def tick(self):
        self.box.update()
        self.craft.update()
        self.tracker.update()
        self.smoother.update()


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop_publisher.publish(Twist())   # as nav2_pidff did on exit: stop the chassis now
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
