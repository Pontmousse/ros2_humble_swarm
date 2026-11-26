"""zero_gate: silent-while-zero wrench publishing must not change what virtual_spacecraft sums."""
from geometry_msgs.msg import Wrench

from swarm_controller.zero_gate import ZeroGatedPublisher

WRENCH_TIMEOUT, DT = 0.25, 0.05  # virtual_spacecraft hold, publisher tick


def wrench(fx=0.0, tz=0.0):
    w = Wrench(); w.force.x, w.torque.z = fx, tz
    return w


class Log:
    def __init__(self): self.msgs, self.t = [], 0.0
    def publish(self, msg): self.msgs.append((self.t, msg.force.x, msg.torque.z))


def summed(log, t):
    """What virtual_spacecraft.total_wrench takes from this source at time t (receive time = publish time)."""
    last = [m for m in log.msgs if m[0] <= t]
    return (last[-1][1], last[-1][2]) if last and t - last[-1][0] <= WRENCH_TIMEOUT else (0.0, 0.0)


def test_one_zero_on_falling_edge_then_silence():
    log = Log(); pub = ZeroGatedPublisher(log)
    for w in (wrench(), wrench(), wrench(0.01), wrench(0.01), wrench(), wrench(), wrench(tz=0.002)):
        pub.publish(w)
    assert [m[1:] for m in log.msgs] == [(0.0, 0.0), (0.01, 0.0), (0.01, 0.0), (0.0, 0.0), (0.0, 0.002)]


def test_virtual_spacecraft_sees_the_same_sum():
    # zero, on, off (long), on, off -- includes a gap longer than wrench_timeout
    pattern = [0.0] * 10 + [0.02] * 7 + [0.0] * 30 + [0.015] * 3 + [0.0] * 12
    gated, full = Log(), Log(); pub = ZeroGatedPublisher(gated)
    for k, fx in enumerate(pattern):
        gated.t = full.t = k * DT
        pub.publish(wrench(fx)); full.publish(wrench(fx))
    assert len(gated.msgs) < len(full.msgs) / 3
    for k in range(len(pattern) * 4):  # virtual_spacecraft ticks at an unrelated phase
        t = k * DT / 4 + 0.003
        assert summed(gated, t) == summed(full, t)
