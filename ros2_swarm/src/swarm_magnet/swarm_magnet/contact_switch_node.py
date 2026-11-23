#!/usr/bin/env python3
"""
Publish `contact_detected` (std_msgs/Bool) from physical limit switches.

Hardware twin of fake_plant's geometric contact check: same topic, same type,
so mode_supervisor's CAPTURE -> DOCK transition needs no change.

Wiring per switch (e.g. V-153-1C25 SPDT): COM -> GND, NO -> GPIO, NC open.
The internal pull-up holds the line HIGH when released; pressing shorts it to
GND (LOW). Lines are requested active-low, so ACTIVE == pressed, and any
pressed switch counts as contact. A broken wire reads as released (fail-safe).
"""

import glob

import gpiod
from gpiod.line import Bias, Direction, Value
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

# Main 40-pin header controller: Pi 5 (RP1), Pi 4 (BCM2711), older Pis. The BCM
# GPIO number is the line offset on each, so GPIO17 is offset 17 everywhere.
# Matched by label because chip numbering differs across boards and kernels.
HEADER_CHIP_LABELS = ('pinctrl-rp1', 'pinctrl-bcm2711', 'pinctrl-bcm2835')


def find_header_chip():
    for path in sorted(glob.glob('/dev/gpiochip*')):
        try:
            with gpiod.Chip(path) as chip:
                if chip.get_info().label in HEADER_CHIP_LABELS:
                    return path
        except OSError:
            continue
    return None


class ContactSwitchNode(Node):
    def __init__(self):
        super().__init__('contact_switch')
        self.declare_parameter('gpio_lines', [17])
        self.declare_parameter('gpio_chip', 'auto')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('debounce_samples', 3)
        self.lines = [int(x) for x in self.get_parameter('gpio_lines').value]
        chip = str(self.get_parameter('gpio_chip').value)
        self.debounce = max(1, int(self.get_parameter('debounce_samples').value))
        self.contact, self.streak, self.request = False, 0, None

        # RELIABLE publishes to both RELIABLE and BEST_EFFORT subscribers.
        self.pub = self.create_publisher(
            Bool, 'contact_detected', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

        chip = find_header_chip() if chip == 'auto' else chip
        if chip is None:
            self.get_logger().error(
                'No Raspberry Pi header GPIO chip found; contact_detected will not be published.')
            return
        self.request = gpiod.request_lines(chip, consumer='ros2_contact_switch', config={
            tuple(self.lines): gpiod.LineSettings(
                direction=Direction.INPUT, bias=Bias.PULL_UP, active_low=True)})
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick)
        self.get_logger().info(
            f'Contact switches on {chip} lines {self.lines} (pull-up, pressed = LOW).')

    def tick(self):
        pressed = any(v == Value.ACTIVE for v in self.request.get_values(self.lines))
        # A new reading must persist for `debounce` samples before it is accepted.
        self.streak = self.streak + 1 if pressed != self.contact else 0
        if self.streak >= self.debounce:
            self.contact, self.streak = pressed, 0
            self.get_logger().info(f'Contact {"DETECTED" if pressed else "released"}')
        # Published every tick (level, not edge) so one lost message is harmless.
        self.pub.publish(Bool(data=self.contact))

    def destroy_node(self):
        if self.request is not None:
            self.request.release()
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
