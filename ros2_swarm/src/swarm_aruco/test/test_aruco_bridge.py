"""aruco_bridge.convert reproduces aruco_detector's Markers contract from the v2 ArucoMarker.

Pure functions only: no node is constructed (the dev host shares a DDS domain with live robots).
"""
import math

import numpy as np
import pytest

from swarm_aruco.aruco_bridge import ArucoBridge, convert
from swarm_interfaces.msg import ArucoMarker, ArucoMarkerArray


def marker(i, t, R):
    """v2 marker: position t (m, camera optical frame), direction = R[:, 2] (marker +Z)."""
    m = ArucoMarker(id=i, distance=float(np.linalg.norm(t)))
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, t)
    m.direction.x, m.direction.y, m.direction.z = map(float, R[:, 2])
    return m


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def detector_reference(t_cm, R, scale):
    """What aruco.pose_estimation + aruco_detector.detect_arucos publish for the same pose (tvec in cm)."""
    z = R[:, 2].copy()
    if z[2] < 0: z *= -1
    n = np.array([z[0], z[2]]); n /= np.linalg.norm(n)
    return (t_cm[2] * scale[0], -t_cm[0] * scale[1]), tuple(n)


@pytest.mark.parametrize('t,yaw', [((0.3, -0.05, 1.2), 0.4), ((-0.5, 0.1, 2.0), -2.6), ((0.0, 0.0, 0.8), math.pi)])
def test_matches_detector_contract(t, yaw):
    R, scale = rot_y(yaw), (1.15, 0.8)
    pos, nrm, ids = convert([marker(7, t, R)], scale, 0.0)
    (x, y), (nx, ny) = detector_reference(100.0 * np.array(t), R, scale)
    assert ids == [7]
    assert (pos[0].x, pos[0].y) == pytest.approx((x, y))
    assert (nrm[0].x, nrm[0].y) == pytest.approx((nx, ny))
    assert nrm[0].y >= 0.0  # forward component forced positive


def test_drops_nan_and_out_of_range_keeps_order():
    nan = marker(1, (math.nan, math.nan, math.nan), np.eye(3))  # uncalibrated camera
    far = marker(2, (0.0, 0.0, 3.0), np.eye(3))
    near = marker(3, (0.1, 0.0, 1.0), np.eye(3))
    pos, nrm, ids = convert([nan, far, near], (1.0, 1.0), 250.0)
    assert ids == [3] and len(pos) == len(nrm) == 1
    assert (pos[0].x, pos[0].y) == pytest.approx((100.0, -10.0))


def test_empty_frame_forwarded_with_header():
    msg = ArucoMarkerArray()
    msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id = 12, 345, 'RM3/camera_optical'
    out = ArucoBridge.to_markers(msg, (1.0, 1.0), 0.0)
    assert out.header == msg.header and list(out.markers) == list(out.markers_normals) == list(out.ids) == []
