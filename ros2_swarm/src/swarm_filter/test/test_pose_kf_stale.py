"""pose_kf withholds its output while wheel odom is dead, and bridges shorter gaps."""
from types import SimpleNamespace

from swarm_filter.pose_kf import PoseKalmanFilterNode


class Stub:
    _tick = PoseKalmanFilterNode._tick

    def __init__(self):
        self.initialized, self.t_ns, self.last_predict_time = True, 0, 0
        self.latest_odom, self.odom_time, self.odometry_timeout = object(), 0, 1.0
        self.predicts, self.published = 0, []

    def _now_ns(self): return self.t_ns
    def _predict(self, dt): self.predicts += 1
    def _publish(self, stamp): self.published.append(self.t_ns)
    def get_logger(self): return SimpleNamespace(warn=lambda *a, **k: None)
    def get_clock(self): return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))


def run(kf, t0, t1):
    for k in range(round((t1 - t0) / 0.05)):
        kf.t_ns = round((t0 + 0.05 * (k + 1)) * 1e9); kf._tick()


def test_gap_shorter_than_timeout_is_bridged():
    kf = Stub(); run(kf, 0.0, 0.9)
    assert len(kf.published) == 18


def test_dead_odom_withholds_output_but_keeps_predicting_then_resumes():
    kf = Stub(); run(kf, 0.0, 3.0)
    assert kf.predicts == 60 and max(kf.published) <= 1.0e9 and len(kf.published) == 20
    kf.odom_time = kf.t_ns  # odom is back
    run(kf, 3.0, 3.1)
    assert len(kf.published) == 22
