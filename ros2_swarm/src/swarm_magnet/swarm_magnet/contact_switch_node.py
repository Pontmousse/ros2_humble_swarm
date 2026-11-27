#!/usr/bin/env python3
"""Standalone contact switch, for a centralized HIL run (switch on the Pi, swarm logic on the PC):
launch_contact_switch.py starts one per robot. On a decentralized Pi the swarm node runs ContactSwitch itself."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from swarm_magnet.contact_switch import ContactSwitch


class ContactSwitchNode(Node):
    def __init__(self):
        super().__init__('contact_switch')
        # RELIABLE publishes to both RELIABLE and BEST_EFFORT subscribers.
        pub = self.create_publisher(Bool, 'contact_detected', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.switch = ContactSwitch(self.get_logger, lambda n, d: self.declare_parameter(n, d).value, pub)
        if self.switch.request is not None:
            self.create_timer(self.switch.period, self.switch.tick)

    def destroy_node(self):
        self.switch.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ContactSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
