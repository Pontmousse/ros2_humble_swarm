"""Map-frame marker memory: lift body-frame Markers into an estimator, project the memory back through a pose.

Moved from landmark_estimator.py (Phase 2, nodes-to-functions); the arithmetic is unchanged.

Frame convention (identical to virtual_scene._gate, so the round-trip is exact):
  * marker POSITION: body-frame centimetres, x=forward, y=left.
  * marker NORMAL: published [X_cam, Z_cam] = [right, forward], forward forced +.
  * pose: (x, y metres, theta DEGREES); th = radians(theta) + cam_offset.
  * body  = R(-th) . (p_map - robot)         (map -> body)
  * p_map = robot + R(th) . body             (body -> map)
"""
import math

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from swarm_interfaces.msg import CoordXY, Markers
from visualization_msgs.msg import Marker, MarkerArray

from swarm_target.estimators import build_estimator


def lift(msg, pose, cam_offset):
    """Body-frame Markers -> [(id, (px, py), (nx, ny))] in the map frame. ids must parallel markers."""
    rx, ry, theta_deg = pose
    th = math.radians(theta_deg) + cam_offset
    c, s = math.cos(th), math.sin(th)
    normals = msg.markers_normals
    have_normals = len(normals) == len(msg.markers)
    out = []
    for i, m in enumerate(msg.markers):
        bx, by = m.x / 100.0, m.y / 100.0
        # [right, fwd] -> body (fwd, left); an unknown normal points along +forward
        nfx, nfy = (normals[i].y, -normals[i].x) if have_normals else (1.0, 0.0)
        out.append((int(msg.ids[i]), (rx + (c * bx - s * by), ry + (s * bx + c * by)),
                    (c * nfx - s * nfy, s * nfx + c * nfy)))
    return out


def project(estimates, pose, cam_offset, gate=None):
    """Map-frame estimates -> body-frame Markers (header left to the caller).

    gate = (max_range_sq, half_fov, cos_incidence) drops what the camera could not see; None = full map.
    """
    rx, ry, theta_deg = pose
    th = math.radians(theta_deg) + cam_offset
    c, s = math.cos(th), math.sin(th)
    out = Markers()
    for est in estimates:
        px, py = float(est.p[0]), float(est.p[1])
        nx, ny = float(est.n[0]), float(est.n[1])
        dx, dy = px - rx, py - ry
        if gate and gate[0] > 0.0 and (dx * dx + dy * dy) > gate[0]:
            continue
        bx, by = c * dx + s * dy, -s * dx + c * dy
        if gate:
            if abs(math.atan2(by, bx)) > gate[1]:
                continue
            dist = math.hypot(dx, dy)
            if dist > 1e-6 and (nx * (-dx) + ny * (-dy)) / dist < gate[2]:
                continue
        nfx, nfy = c * nx + s * ny, -s * nx + c * ny
        if nfx < 0.0:
            nfx, nfy = -nfx, -nfy
        out.markers.append(CoordXY(x=bx * 100.0, y=by * 100.0))
        out.markers_normals.append(CoordXY(x=-nfy, y=nfx))   # [X_cam right, Z_cam forward]
        out.ids.append(int(est.id))
    return out


class MarkerMemory:
    """One remembered marker set (landmarks or targets): frozen-stamp gate + estimator + lift/project."""

    def __init__(self, estimator_type="averaging", min_observations=2, forget_timeout=0.0, camera_yaw_offset_deg=0.0,
                 output_fov_gate=False, output_max_range=0.0, half_fov_deg=40.0, max_incidence_deg=70.0):
        self.estimator = build_estimator(estimator_type, min_observations=min_observations,
                                         forget_timeout=forget_timeout)
        self.cam_offset = math.radians(camera_yaw_offset_deg)
        self.gate = (output_max_range ** 2, math.radians(half_fov_deg),
                     math.cos(math.radians(max_incidence_deg))) if output_fov_gate else None
        self.last_stamp = None

    def observe(self, msg, pose, t):
        """Fuse one frame. Returns the number of markers fused, or -1 when ids do not parallel markers.

        Only genuinely fresh frames count: a source that republishes a held frame keeps its stamp FROZEN, and
        fusing those would over-weight one observation.
        """
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return 0
        self.last_stamp = stamp
        if pose is None or not msg.markers:
            return 0
        if len(msg.ids) != len(msg.markers):
            return -1
        for mid, p, n in lift(msg, pose, self.cam_offset):
            self.estimator.observe(mid, p, n, t)
        return len(msg.markers)

    def predict(self, dt):
        self.estimator.predict(dt)

    def estimates(self):
        return self.estimator.landmarks()

    def project(self, pose, estimates=None):
        return project(self.estimates() if estimates is None else estimates, pose, self.cam_offset, self.gate)


def memory_viz(estimates, stamp, frame_id, ns):
    """Map-frame debug markers: a sphere, the normal and an `id(n=count)` label per remembered marker."""
    arr = MarkerArray()
    arr.markers.append(Marker(action=Marker.DELETEALL))
    arr.markers[0].header.frame_id, arr.markers[0].header.stamp = frame_id, stamp
    for k, est in enumerate(estimates):
        px, py = float(est.p[0]), float(est.p[1])
        nx, ny = float(est.n[0]), float(est.n[1])
        sphere = Marker(ns=ns, id=3 * k, type=Marker.SPHERE, action=Marker.ADD, scale=Vector3(x=0.08, y=0.08, z=0.08),
                        color=ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.4 + 0.5 * float(min(1.0, est.confidence))))
        sphere.pose.position = Point(x=px, y=py, z=0.0)
        sphere.pose.orientation.w = 1.0
        normal = Marker(ns=ns, id=3 * k + 1, type=Marker.LINE_LIST, action=Marker.ADD,
                        color=ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.9),
                        points=[Point(x=px, y=py, z=0.0), Point(x=px + 0.08 * nx, y=py + 0.08 * ny, z=0.0)])
        normal.scale.x = 0.01
        # No spaces: RViz2 Humble draws a space ~0.6 m wide regardless of scale.z.
        label = Marker(ns=ns, id=3 * k + 2, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD,
                       color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9), text=f"{est.id}(n={est.count})")
        label.pose.position = Point(x=px, y=py, z=0.12)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.06
        for m in (sphere, normal, label):
            m.header.frame_id, m.header.stamp = frame_id, stamp
            arr.markers.append(m)
    return arr
