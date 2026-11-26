"""mode_supervisor service calls: never block, never drop, resend until the server confirms."""
from types import SimpleNamespace

from rclpy.task import Future
from rclpy.time import Time
from std_srvs.srv import SetBool

from swarm_controller.mode_supervisor import ModeSupervisor


class Client:
    def __init__(self, name, ready=True):
        self.srv_name, self.ready, self.requests = name, ready, []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        f = Future(); self.requests.append((request.data, f)); log.append((self.srv_name, request.data))
        return f

    def reply(self, k=-1, success=True):
        self.requests[k][1].set_result(SetBool.Response(success=success))  # no executor: callbacks run now


log = []


class Supervisor:
    call, flush, on_reply = ModeSupervisor.call, ModeSupervisor.flush, ModeSupervisor.on_reply

    def __init__(self):
        self.pending, self.sent, self.retry, self.t = {}, {}, {}, 100.0

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(nanoseconds=round(self.t * 1e9)))

    def get_logger(self):
        return SimpleNamespace(warn=lambda *a, **k: None)


def test_late_server_is_still_called():
    s, c = Supervisor(), Client("encapsulation/enable", ready=False)
    s.call(c, True); s.flush()
    assert c.requests == []
    c.ready = True; s.t += 0.2; s.flush()
    assert [r[0] for r in c.requests] == [True]
    c.reply(); s.t += 5.0; s.flush()
    assert len(c.requests) == 1 and s.pending == {}


def test_lost_reply_is_resent_after_retry_period():
    s, c = Supervisor(), Client("toggle_controller")
    s.call(c, True); s.flush(); s.t += 0.8; s.flush()
    assert len(c.requests) == 1
    s.t += 0.3; s.flush()
    assert len(c.requests) == 2
    c.reply(0, success=False); c.reply(1)
    assert s.pending == {}


def test_newer_state_wins_and_stale_reply_is_ignored():
    s, c = Supervisor(), Client("toggle_dock")
    s.call(c, True); s.flush()
    s.call(c, False); s.flush()
    c.reply(0)  # the True confirmation must not clear the pending False
    assert s.pending == {c: False}
    c.reply(1)
    assert s.pending == {}


def test_calls_go_out_in_call_order():
    log.clear()
    s = Supervisor()
    wheels, old, new = Client("engage_wheels"), Client("toggle_controller"), Client("toggle_dock")
    s.call(wheels, False); s.call(old, False); s.call(new, True); s.flush()
    assert log == [("engage_wheels", False), ("toggle_controller", False), ("toggle_dock", True)]
