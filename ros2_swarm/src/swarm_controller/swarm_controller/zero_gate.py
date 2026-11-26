"""Wrench publisher that stays silent while its output is zero.

virtual_spacecraft holds each wrench source for wrench_timeout (0.25 s) after its last message, then
drops it from the sum, so a silent source IS a zero source. A zero therefore needs to go out once, on
the falling edge (it replaces a held non-zero value at once), and never again; anything non-zero goes
out every tick. The wrench topics are RELIABLE, so that single zero is delivered.
"""


def is_zero(w):
    return not any((w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z))


class ZeroGatedPublisher:
    def __init__(self, publisher): self.publisher, self.zero_sent = publisher, False

    def publish(self, msg):
        zero = is_zero(msg)
        if zero and self.zero_sent: return
        self.publisher.publish(msg)
        self.zero_sent = zero
