"""VelocitySmoother (Python port) against the real nav2 Humble velocity_smoother.

golden_smoother_nav2.json was recorded from the installed C++ node (our yaml limits, 10 Hz) fed scripted
cmd_vel at 20 Hz, each command held 1 s, with one 1.5 s silence. OPEN_LOOP makes every output a function of
the previous output and the latest command, so each recorded output must be reproduced from those. Right after
a command switch (or near the timeout) either neighbouring command may have been the latest; any must match.
"""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist

from swarm_nav2_controller.smoother import VelocitySmoother

FIXTURE = json.loads((Path(__file__).parent / 'golden_smoother_nav2.json').read_text())


class Clock:
    def __init__(self): self.ns = 0
    def now(self): return SimpleNamespace(nanoseconds=self.ns)


def smoother(frequency=10.0, **over):
    clock, out = Clock(), []
    params = {**FIXTURE['params'], **over}
    node = SimpleNamespace(get_clock=lambda: clock, get_logger=lambda: SimpleNamespace(error=lambda *a: None))
    s = VelocitySmoother(node, lambda n, d: params.get(n, d), SimpleNamespace(publish=out.append), frequency)
    return s, clock, out


def twist(x, y, wz):
    t = Twist(); t.linear.x, t.linear.y, t.angular.z = float(x), float(y), float(wz)
    return t


def step(prev, command):
    """One smoother tick from last_cmd=prev with command latest (None = timed out)."""
    s, clock, out = smoother()
    s.last_cmd = list(prev)
    s.command_callback(twist(*(command or (0.0, 0.0, 0.0))))
    if command is None: clock.ns = 10**9                                  # past velocity_timeout
    s.update()
    return [out[0].linear.x, out[0].linear.y, out[0].angular.z] if out else None


def test_port_reproduces_every_nav2_output():
    cmds, outs = FIXTURE['commands'], FIXTURE['outputs']
    prev = [0.0, 0.0, 0.0]
    for t, *expected in outs:
        recent = [c[1:] for c in cmds if t - 0.16 < c[0] <= t]
        before = [c[1:] for c in cmds if c[0] <= t - 0.16][-1:]
        candidates = [tuple(c) for c in recent + before] + ([None] if not cmds or t - max(
            (c[0] for c in cmds if c[0] <= t), default=-1) > 0.15 else [])
        got = [step(prev, c) for c in dict.fromkeys(candidates)]
        assert any(g is not None and g == pytest.approx(expected, abs=1e-9) for g in got), (t, expected, got)
        prev = expected
    assert len(outs) > 80


def test_timeout_decelerates_then_latches_silent():
    s, clock, out = smoother()
    s.command_callback(twist(0.5, 0.0, 0.0))
    for _ in range(4): s.update()                                         # 0.15/step at 1.5 m/s^2, 10 Hz
    assert out[-1].linear.x == pytest.approx(0.5)
    clock.ns = 3 * 10**8                                                  # 0.3 s > velocity_timeout 0.25
    for _ in range(6): s.update()
    assert [round(m.linear.x, 9) for m in out[4:]] == [0.3, 0.1, 0.0]   # decel 2.0 m/s^2, then silence
    s.command_callback(twist(0.2, 0.0, 0.0)); s.update()
    assert out[-1].linear.x == pytest.approx(0.15)                        # a new command unlatches


def test_twenty_hz_keeps_the_same_limits_per_second():
    for f in (10.0, 20.0):
        s, _, out = smoother(f)
        s.command_callback(twist(1.0, 0.0, 0.0))
        for _ in range(int(f * 0.4)): s.update()                          # 0.4 s of ticks
        assert out[-1].linear.x == pytest.approx(0.6)                     # 1.5 m/s^2 * 0.4 s
