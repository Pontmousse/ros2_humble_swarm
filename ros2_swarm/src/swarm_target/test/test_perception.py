"""perception (Phase 2): marker memory maths, pose history, and the node's tick wiring -- without joining DDS."""
import math
from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from swarm_interfaces.msg import ArucoMarker, ArucoMarkerArray, CoordXY, Markers

from swarm_filter.pose_kf import PoseKalmanFilterNode
from swarm_target.marker_memory import MarkerMemory, lift, project
from swarm_target.perception_node import PerceptionNode
from swarm_target.pose_history import PoseHistory


def frame(sec, *markers):
    """Markers frame; each marker (x_cm, y_cm, id, right, fwd)."""
    m = Markers()
    m.header.stamp = Time(sec=sec)
    for x, y, i, nr, nf in markers:
        m.markers.append(CoordXY(x=float(x), y=float(y)))
        m.markers_normals.append(CoordXY(x=float(nr), y=float(nf)))
        m.ids.append(i)
    return m


def test_lift_matches_the_frame_convention():
    # Robot at (1, 2) facing +y (90 deg): 1 m ahead is (1, 3); published normal [right, fwd] = [0, 1] is
    # body (fwd 1, left 0) -> map +y.
    [(mid, p, n)] = lift(frame(1, (100.0, 0.0, 7, 0.0, 1.0)), (1.0, 2.0, 90.0), 0.0)
    assert mid == 7 and p == pytest.approx((1.0, 3.0)) and n == pytest.approx((0.0, 1.0), abs=1e-12)


@pytest.mark.parametrize("pose", [(0.0, 0.0, 0.0), (1.2, -0.4, 137.0), (-2.0, 3.0, -75.0)])
def test_project_inverts_lift(pose):
    mem = MarkerMemory(min_observations=1)
    raw = frame(1, (150.0, 20.0, 3, -0.2, 0.98), (90.0, -60.0, 5, 0.3, 0.95))
    assert mem.observe(raw, pose, 1.0) == 2
    out = mem.project(pose)
    assert list(out.ids) == [3, 5]
    for a, b in zip(out.markers, raw.markers):
        assert (a.x, a.y) == pytest.approx((b.x, b.y))
    for a, b in zip(out.markers_normals, raw.markers_normals):
        assert (a.x, a.y) == pytest.approx(tuple(np.array([b.x, b.y]) / math.hypot(b.x, b.y)))


def test_frozen_stamp_is_fused_once_and_bad_ids_are_refused():
    mem = MarkerMemory(min_observations=2)
    f = frame(4, (100.0, 0.0, 1, 0.0, 1.0))
    assert mem.observe(f, (0.0, 0.0, 0.0), 1.0) == 1
    assert mem.observe(f, (0.0, 0.0, 0.0), 1.1) == 0           # same stamp: a republished frame
    assert mem.estimates() == []                               # still below min_observations
    assert mem.observe(frame(5, (100.0, 0.0, 1, 0.0, 1.0)), None, 1.2) == 0   # no pose yet
    bad = frame(6, (100.0, 0.0, 1, 0.0, 1.0)); bad.ids.append(9)
    assert mem.observe(bad, (0.0, 0.0, 0.0), 1.3) == -1


def test_project_gate_drops_what_the_camera_could_not_see():
    mem = MarkerMemory(min_observations=1, output_fov_gate=True, output_max_range=3.0, half_fov_deg=40.0)
    # normals point back at the robot (the incidence test's convention): id 1 faces the camera head-on
    mem.observe(frame(1, (200.0, 0.0, 1, 0.0, -1.0), (0.0, 150.0, 2, 0.0, -1.0)), (0.0, 0.0, 0.0), 1.0)
    assert list(mem.project((0.0, 0.0, 0.0)).ids) == [1]       # id 2 sits 90 deg off the camera axis


def test_pose_history_interpolates_and_falls_back():
    h = PoseHistory(span=2.0)
    for k in range(5):
        h.add(10.0 + 0.05 * k, (0.1 * k, 0.0, 170.0 + 5.0 * k))   # heading crosses +180
    x, _, th = h.at(10.125, "latest")
    assert x == pytest.approx(0.25) and (th - 182.5) % 360.0 == pytest.approx(0.0, abs=1e-9)
    assert h.at(9.0, "latest") == "latest" and h.at(10.3, "latest") == "latest"


class Pub:
    def __init__(self): self.msgs = []
    def publish(self, m): self.msgs.append(m)


def node(monkeypatch, viz_period=0.0):
    """A PerceptionNode without __init__ (no DDS); the KF base publish is recorded instead of sent."""
    kf = []
    monkeypatch.setattr(PoseKalmanFilterNode, "_publish", lambda self, stamp: kf.append(stamp))
    n = PerceptionNode.__new__(PerceptionNode)
    n.xp, n.xh = np.array([1.0, 2.0, 0.0, 0.0]), np.array([math.pi / 2, 0.0])
    n.pose_now, n.last_t, n.last_viz, n.viz_period, n.lift_at_stamp = None, None, 0.0, viz_period, False
    n.history, n.global_frame_id, n.t_ns = PoseHistory(), "swarm_map", 0
    n.sets = [(name, MarkerMemory(min_observations=1), Pub(), Pub()) for name in ("landmarks", "targets")]
    n._now_ns = lambda: n.t_ns
    n.get_logger = lambda: SimpleNamespace(warn=lambda *a, **k: None)
    return n, kf


def test_markers_are_projected_through_the_pose_just_published(monkeypatch):
    n, kf = node(monkeypatch)
    n._publish(Time(sec=10))                                    # first tick: pose known, memory still empty
    n._on_markers(n.sets[1][1], frame(10, (100.0, 0.0, 4, 0.0, 1.0)))
    n.xp[:2] = (1.0, 2.5)                                       # robot moves 0.5 m toward the target
    n._publish(Time(sec=10, nanosec=50_000_000))
    out = n.sets[1][2].msgs[-1]
    assert len(kf) == 2 and out.header.stamp == Time(sec=10, nanosec=50_000_000) and out.header.frame_id == "swarm_map"
    assert list(out.ids) == [4] and (out.markers[0].x, out.markers[0].y) == pytest.approx((50.0, 0.0))
    assert n.sets[0][2].msgs[-1].markers == []                  # landmarks: empty memory, still published


def test_nothing_is_projected_while_the_kf_withholds(monkeypatch):
    n, kf = node(monkeypatch)
    n.initialized, n.last_predict_time, n.latest_odom, n.odom_time, n.odometry_timeout = True, 0, object(), 0, 1.0
    n._predict = lambda dt: None
    n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=2)))
    n.t_ns = 2_000_000_000                                       # odom silent for 2 s > odometry_timeout
    n._tick()
    assert kf == [] and all(s[2].msgs == [] for s in n.sets)


def test_v2_frames_are_republished_raw_and_fused(monkeypatch):
    n, _ = node(monkeypatch)
    n._publish(Time(sec=1))
    arr = ArucoMarkerArray()
    arr.header.stamp = Time(sec=1, nanosec=10)
    m = ArucoMarker(id=12)
    m.pose.position.x, m.pose.position.z = 0.0, 1.5           # 1.5 m straight ahead (optical frame)
    m.direction.z = -1.0                                        # marker faces the camera
    arr.markers.append(m)
    raw = Pub()
    from swarm_aruco.aruco_convert import to_markers
    n._on_aruco(n.sets[1][1], to_markers(arr, (1.0, 1.0), 0.0), raw)
    assert list(raw.msgs[0].ids) == [12] and raw.msgs[0].header.stamp == arr.header.stamp
    [(mid, p, _)] = [(e.id, e.p, e.n) for e in n.sets[1][1].estimates()]
    assert mid == 12 and tuple(p) == pytest.approx((1.0, 3.5))  # robot (1, 2) facing +y


def test_memory_viz_is_throttled(monkeypatch):
    n, _ = node(monkeypatch, viz_period=0.5)
    for k in range(20):                                         # 1 s of 20 Hz ticks
        n._publish(Time(sec=100, nanosec=50_000_000 * k))
    assert len(n.sets[0][3].msgs) == 2
