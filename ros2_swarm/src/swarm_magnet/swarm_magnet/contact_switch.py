"""Contact limit switches on the Pi header GPIO -> `contact_detected` (std_msgs/Bool). Plain class driven by the
swarm node (contact.source: gpio) or by the standalone contact_switch_node (centralized HIL).

Hardware twin of geometric_contact.py: same topic, same type; mode_supervisor
ORs the two, so both may run at once.

Wiring per switch (e.g. V-153-1C25 SPDT): COM -> GND, NO -> GPIO, NC open.
The internal pull-up holds the line HIGH when released; pressing shorts it to
GND (LOW). Lines are requested active-low, so ACTIVE == pressed, and any
pressed switch counts as contact. A broken wire reads as released (fail-safe).
gpiod is imported only here, so the swarm node runs without it in SIL.
"""

import glob

import gpiod
from gpiod.line import Bias, Direction, Value
from std_msgs.msg import Bool

# Main 40-pin header controller: Pi 5 (RP1), Pi 4 (BCM2711), older Pis. The BCM
# GPIO number is the line offset on each, so GPIO5 is offset 5 everywhere.
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

class ContactSwitch:
    def __init__(self, logger, p, pub):
        """p(name, default) reads gpio_lines, gpio_chip, rate_hz, debounce_samples; pub publishes contact_detected."""
        self.get_logger = logger
        self.lines = [int(x) for x in p('gpio_lines', [5])]
        chip = str(p('gpio_chip', 'auto'))
        self.period = 1.0 / float(p('rate_hz', 20.0))  # level stream (the DOCK latch needs True while in CAPTURE); 3-sample debounce = 150 ms
        self.debounce = max(1, int(p('debounce_samples', 3)))
        self.contact, self.streak, self.request = False, 0, None
        self.pub = pub
        chip = find_header_chip() if chip == 'auto' else chip
        if chip is None:
            self.get_logger().error(
                'No Raspberry Pi header GPIO chip found; contact_detected will not be published.')
            return
        self.request = gpiod.request_lines(chip, consumer='ros2_contact_switch', config={
            tuple(self.lines): gpiod.LineSettings(
                direction=Direction.INPUT, bias=Bias.PULL_UP, active_low=True)})
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

    def release(self):
        if self.request is not None:
            self.request.release()
