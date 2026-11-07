"""Geometric perception simulator -- a drop-in replacement for the camera.

Centralized runs on the dev machine, where the robots' cameras are unreachable,
so the ArUco detectors get no frames and the mission can't be exercised. This one
global node owns a geometric world and a per-robot sensor model (range + FOV +
facing), and publishes the SAME final topics the real pipeline would --
`/<ns>/landmarks` and `/<ns>/targets` (swarm_interfaces/Markers) -- so
mode_supervisor, the ep/cp controllers and the virtual spacecraft run unchanged.

Object model: each scene body is ONE physical object with an arbitrary 2D
boundary (convex/concave/curved/polygonal/irregular). BOTH kinds of points live
on that same perimeter:
  * landmarks     -- dense reference points describing the whole boundary
                     (encapsulation surrounds the object with these),
  * capture points -- a few attachment points auto-placed on edge interiors, away
                     from corners (the capture controller docks to one of these).
Each point carries an outward normal. Geometry is defined once (the outline);
landmark density and capture placement are independent of it.

Conventions (verified against swarm_aruco):
  * Marker POSITION is body-frame centimetres, x=forward, y=left
    (aruco.py: x = tvec_z, y = -tvec_x).
  * Marker NORMAL is published as [X_cam, Z_cam] = [right, forward], flipped so
    the forward component is positive (aruco_detector projects the marker z-axis
    and forces z>=0). cp_proportional_controller reads markers_normals[closest]
    and drives the right component to zero to face the marker head-on.
  * Pose is /<ns>/pose = StatePos (x,y metres; theta DEGREES), the same source
    the controllers use, so projection errors cancel and the sim is consistent.

Capture points use a SMALLER detection range than landmarks (target_max_range),
modelling that the fine capture markers only resolve up close -- this restores the
staged SEARCH->ENCAP->CAPTURE sequence for a single object.

A second output, /virtual_scene/markers (visualization_msgs/MarkerArray, frame
swarm_map), draws the ground-truth world for RViz (white outline, red landmarks,
blue capture points, each with normals), distinct from the per-robot detected view.
"""

import math
import os
import random

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from swarm_interfaces.msg import CoordXY, Markers, StatePos


_OUTLINE_COLOR = (0.85, 0.85, 0.85)   # white  -- the solid object boundary
_LANDMARK_COLOR = (0.9, 0.1, 0.1)     # red    -- dense boundary references
_CAPTURE_COLOR = (0.1, 0.35, 0.95)    # blue   -- selected attachment points
_NORMAL_LEN = 0.08                    # metres, RViz normal arrows/lines


def _rot(cos_a, sin_a, x, y):
    """Rotate (x, y) by the angle whose cos/sin are given."""
    return cos_a * x - sin_a * y, sin_a * x + cos_a * y


def _centroid(outline):
    cx = sum(p[0] for p in outline) / len(outline)
    cy = sum(p[1] for p in outline) / len(outline)
    return cx, cy


def _outward_normal(p0, p1, cx, cy):
    """Unit normal of edge p0->p1, flipped to point away from the centroid."""
    ex, ey = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(ex, ey)
    if length < 1e-9:
        return None, None, 0.0
    nx, ny = ey / length, -ex / length
    mx, my = (p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0
    if (mx - cx) * nx + (my - cy) * ny < 0.0:
        nx, ny = -nx, -ny
    return nx, ny, length


class Object:
    """One rigid object: a boundary that yields dense landmarks + sparse captures.

    `landmark_points`/`landmark_normals` are the dense outline samples that
    impersonate the printed ArUco sheet; `capture_points`/`capture_normals` are
    the few auto-placed attachment points. `outline` is the sparse corner polygon
    used for the RViz boundary. All are in the body frame; `com`/`yaw` are
    integrated in the map frame each tick.
    """

    def __init__(self, spec):
        self.com = list(spec.get("com", [0.0, 0.0]))
        self.yaw = math.radians(float(spec.get("yaw_deg", 0.0)))
        self.vel = list(spec.get("vel", [0.0, 0.0]))
        self.omega = math.radians(float(spec.get("omega_deg", 0.0)))
        ppm = float(spec.get("landmark_points_per_m", 20.0))

        self.outline = self._outline(spec)
        self.landmark_points, self.landmark_normals = self._sample(
            self.outline, ppm)
        self.capture_points, self.capture_normals = self._captures(
            self.outline, spec)

    # -- geometry ---------------------------------------------------------
    @staticmethod
    def _outline(spec):
        """Closed corner polygon (body frame) from the shape spec."""
        verts = spec.get("vertices") or []
        if verts:
            return [(float(x), float(y)) for x, y in verts]

        shape = str(spec.get("shape", "square")).lower()
        size = [float(s) for s in spec.get("size", [0.3])]
        if shape == "circle":
            r = size[0]
            n = max(8, int(round(2.0 * math.pi * r * 16)))
            return [(r * math.cos(2 * math.pi * k / n),
                     r * math.sin(2 * math.pi * k / n)) for k in range(n)]
        # square / rectangle
        w = size[0]
        h = size[1] if len(size) > 1 else size[0]
        hw, hh = w / 2.0, h / 2.0
        return [(hw, -hh), (hw, hh), (-hw, hh), (-hw, -hh)]

    @staticmethod
    def _sample(outline, ppm):
        """Walk the closed outline at 1/ppm spacing -> dense points + normals."""
        cx, cy = _centroid(outline)
        spacing = 1.0 / max(ppm, 1e-6)
        points, normals = [], []
        n = len(outline)
        for i in range(n):
            p0, p1 = outline[i], outline[(i + 1) % n]
            nx, ny, length = _outward_normal(p0, p1, cx, cy)
            if length < 1e-9:
                continue
            ex, ey = p1[0] - p0[0], p1[1] - p0[1]
            steps = max(1, int(round(length / spacing)))
            for s in range(steps):
                t = (s + 0.5) / steps
                points.append((p0[0] + t * ex, p0[1] + t * ey))
                normals.append((nx, ny))
        return points, normals

    @staticmethod
    def _captures(outline, spec):
        """Auto-place a few capture points on the boundary, away from corners.

        Default: one per edge midpoint (skipping edges too short to clear the
        corner margin). `capture_count` overrides to that many points evenly by
        arc length (needed for circles/curves). `captures: none` disables.
        """
        mode = str(spec.get("captures", "auto")).lower()
        if mode == "none":
            return [], []
        cx, cy = _centroid(outline)
        count = spec.get("capture_count")
        if count is not None:
            return Object._captures_by_arclength(outline, int(count), cx, cy)
        per_edge = int(spec.get("captures_per_edge", 1))
        margin = float(spec.get("corner_margin_m", 0.03))
        return Object._captures_per_edge(outline, per_edge, margin, cx, cy)

    @staticmethod
    def _captures_per_edge(outline, per_edge, margin, cx, cy):
        points, normals = [], []
        n = len(outline)
        for i in range(n):
            p0, p1 = outline[i], outline[(i + 1) % n]
            nx, ny, length = _outward_normal(p0, p1, cx, cy)
            usable = length - 2.0 * margin
            if usable <= 1e-6:
                continue  # edge too short to host a capture clear of corners
            ex, ey = p1[0] - p0[0], p1[1] - p0[1]
            for k in range(per_edge):
                frac = 0.5 if per_edge == 1 else k / (per_edge - 1)
                t = (margin + usable * frac) / length
                points.append((p0[0] + t * ex, p0[1] + t * ey))
                normals.append((nx, ny))
        return points, normals

    @staticmethod
    def _captures_by_arclength(outline, count, cx, cy):
        edges, total = [], 0.0
        n = len(outline)
        for i in range(n):
            p0, p1 = outline[i], outline[(i + 1) % n]
            nx, ny, length = _outward_normal(p0, p1, cx, cy)
            if length < 1e-9:
                continue
            edges.append((total, length, p0, p1, nx, ny))
            total += length
        points, normals = [], []
        for k in range(max(0, count)):
            s_target = (k + 0.5) * total / count   # half-step -> never on a vertex
            for s0, length, p0, p1, nx, ny in edges:
                if s_target <= s0 + length or (s0, p0) == (edges[-1][0], edges[-1][2]):
                    local = (s_target - s0) / length
                    ex, ey = p1[0] - p0[0], p1[1] - p0[1]
                    points.append((p0[0] + local * ex, p0[1] + local * ey))
                    normals.append((nx, ny))
                    break
        return points, normals

    # -- motion -----------------------------------------------------------
    def integrate(self, dt):
        self.com[0] += self.vel[0] * dt
        self.com[1] += self.vel[1] * dt
        self.yaw += self.omega * dt

    def _transform(self, points, normals):
        """(body points, body normals) -> list of (map point, map normal)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        cx, cy = self.com
        out = []
        for (px, py), (nx, ny) in zip(points, normals):
            rx, ry = _rot(c, s, px, py)
            nrx, nry = _rot(c, s, nx, ny)
            out.append(((cx + rx, cy + ry), (nrx, nry)))
        return out

    def world_landmarks(self):
        return self._transform(self.landmark_points, self.landmark_normals)

    def world_captures(self):
        return self._transform(self.capture_points, self.capture_normals)

    def world_outline(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        cx, cy = self.com
        return [(cx + _rot(c, s, px, py)[0], cy + _rot(c, s, px, py)[1])
                for px, py in self.outline]


class VirtualScene(Node):
    def __init__(self):
        super().__init__("virtual_scene")

        self.declare_parameter("robot_names", [""])
        self.declare_parameter("scene_file", "")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("max_range", 1.0)
        self.declare_parameter("target_max_range", 0.0)  # 0 -> fall back to max_range
        self.declare_parameter("half_fov_deg", 40.0)
        self.declare_parameter("max_incidence_deg", 70.0)
        self.declare_parameter("camera_yaw_offset_deg", 0.0)
        self.declare_parameter("noise_std_cm", 0.0)
        self.declare_parameter("global_frame_id", "swarm_map")

        # -- raw-observation sim mode (feeds the landmark_estimator) ----------
        # When emit_raw_observations is on, publish hardware-like noisy / dropped
        # / starved detections on the *_unf topics (the estimator's input), with
        # STABLE per-point ArUco-style IDs tied to ground truth. publish_fused
        # keeps the clean landmarks/targets for the legacy no-estimator path.
        self.declare_parameter("emit_raw_observations", False)
        self.declare_parameter("publish_fused", True)
        self.declare_parameter("normal_noise_std", 0.0)       # unit-vector jitter
        self.declare_parameter("observation_dropout_prob", 0.0)  # per-marker skip
        self.declare_parameter("obs_update_rate", 6.0)        # Hz of genuine fixes

        names = list(self.get_parameter("robot_names").value)
        self.robot_names = [n for n in names if n]
        scene_file = str(self.get_parameter("scene_file").value)
        rate = float(self.get_parameter("publish_rate").value)
        self.max_range = float(self.get_parameter("max_range").value)
        tmr = float(self.get_parameter("target_max_range").value)
        self.target_max_range = tmr if tmr > 0.0 else self.max_range
        self.half_fov = math.radians(
            float(self.get_parameter("half_fov_deg").value))
        self.cos_incidence = math.cos(math.radians(
            float(self.get_parameter("max_incidence_deg").value)))
        self.cam_offset = math.radians(
            float(self.get_parameter("camera_yaw_offset_deg").value))
        self.noise_std = float(self.get_parameter("noise_std_cm").value)
        self.frame_id = str(self.get_parameter("global_frame_id").value)

        self.emit_raw = bool(self.get_parameter("emit_raw_observations").value)
        self.publish_fused = bool(self.get_parameter("publish_fused").value)
        self.normal_noise_std = float(
            self.get_parameter("normal_noise_std").value)
        self.dropout_prob = float(
            self.get_parameter("observation_dropout_prob").value)
        obs_rate = float(self.get_parameter("obs_update_rate").value)
        self._obs_period = 1.0 / max(obs_rate, 1e-6)
        self._last_obs_time = None          # None -> first tick is a genuine fix
        self._raw_hold = {}                 # name -> (landmarks_msg, targets_msg)

        self.objects = self._load_scene(scene_file)

        # Stable ground-truth IDs, one per world landmark / capture point, in the
        # same concatenation order tick() builds world_landmarks / world_captures.
        # Landmarks and targets are independent streams, so their IDs may overlap
        # safely; keep distinct bases anyway for readable debugging.
        self.landmark_ids = []
        lid = 10
        for o in self.objects:
            for _ in o.landmark_points:
                self.landmark_ids.append(lid)
                lid += 1
        self.capture_ids = []
        cid = 1000
        for o in self.objects:
            for _ in o.capture_points:
                self.capture_ids.append(cid)
                cid += 1

        # Publisher QoS == aruco_filter's effective QoS, so this is a drop-in
        # replacement compatible with every (BEST_EFFORT or RELIABLE) subscriber.
        pub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Subscriber QoS BEST_EFFORT -> compatible with any pose publisher.
        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.poses = {}          # robot name -> (x, y, theta_deg)
        self.landmark_pubs = {}
        self.target_pubs = {}
        self.landmark_unf_pubs = {}
        self.target_unf_pubs = {}
        for name in self.robot_names:
            self.landmark_pubs[name] = self.create_publisher(
                Markers, f"/{name}/landmarks", pub_qos)
            self.target_pubs[name] = self.create_publisher(
                Markers, f"/{name}/targets", pub_qos)
            if self.emit_raw:
                # RELIABLE publisher is compatible with the estimator's
                # subscription whether it is configured RELIABLE or BEST_EFFORT.
                self.landmark_unf_pubs[name] = self.create_publisher(
                    Markers, f"/{name}/landmarks_unf", pub_qos)
                self.target_unf_pubs[name] = self.create_publisher(
                    Markers, f"/{name}/targets_unf", pub_qos)
            self.create_subscription(
                StatePos, f"/{name}/pose",
                lambda msg, n=name: self._on_pose(n, msg), sub_qos)

        self.viz_pub = self.create_publisher(
            MarkerArray, "/virtual_scene/markers", 1)

        self.last_time = self.get_clock().now()
        self.create_timer(1.0 / max(rate, 1e-6), self.tick)

        mode = ("raw->*_unf" if self.emit_raw else "") + \
               (" + fused" if self.publish_fused else "")
        self.get_logger().info(
            f"virtual_scene: {len(self.objects)} objects, robots "
            f"{self.robot_names}, landmark_range={self.max_range} m, "
            f"target_range={self.target_max_range} m, "
            f"half_fov={math.degrees(self.half_fov):.0f} deg, "
            f"mode=[{mode.strip() or 'fused'}].")

    # -- setup ------------------------------------------------------------
    def _load_scene(self, path):
        if not path or not os.path.isfile(path):
            self.get_logger().error(f"scene_file not found: {path!r}")
            return []
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        objects = [Object(spec) for spec in data.get("objects", [])]
        for o in objects:
            self.get_logger().info(
                f"  object at com {o.com}: {len(o.landmark_points)} landmarks, "
                f"{len(o.capture_points)} capture points.")
        return objects

    def _on_pose(self, name, msg: StatePos):
        self.poses[name] = (msg.x, msg.y, msg.theta)

    # -- main loop --------------------------------------------------------
    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        for o in self.objects:
            o.integrate(dt)

        # Snapshot every object's world geometry once per tick.
        world_landmarks, world_captures = [], []
        for o in self.objects:
            world_landmarks.extend(o.world_landmarks())
            world_captures.extend(o.world_captures())

        # Clean fused output (legacy no-estimator path): always fresh stamp.
        if self.publish_fused:
            for name in self.robot_names:
                pose = self.poses.get(name)
                if pose is None:
                    continue  # skip until the first pose arrives
                self.landmark_pubs[name].publish(self._gate(
                    pose, world_landmarks, self.landmark_ids, self.max_range))
                self.target_pubs[name].publish(self._gate(
                    pose, world_captures, self.capture_ids,
                    self.target_max_range))

        # Raw hardware-like observations feeding the estimator: noisy / dropped,
        # and STARVED -- a genuine fix mints a new stamp at obs_update_rate, while
        # the timer keeps re-publishing the held frame with that FROZEN stamp in
        # between, reproducing the real detector's ~0.3 Hz true-detection rate and
        # exercising the estimator's fresh-gate.
        if self.emit_raw:
            genuine = (self._last_obs_time is None or
                       (now - self._last_obs_time).nanoseconds * 1e-9
                       >= self._obs_period)
            if genuine:
                self._last_obs_time = now
            for name in self.robot_names:
                pose = self.poses.get(name)
                if pose is None:
                    continue
                if genuine:
                    lm = self._gate(pose, world_landmarks, self.landmark_ids,
                                    self.max_range, raw=True)
                    tg = self._gate(pose, world_captures, self.capture_ids,
                                    self.target_max_range, raw=True)
                    self._raw_hold[name] = (lm, tg)
                else:
                    held = self._raw_hold.get(name)
                    if held is None:
                        continue  # no genuine fix yet to hold
                    lm, tg = held  # re-publish with its FROZEN stamp
                self.landmark_unf_pubs[name].publish(lm)
                self.target_unf_pubs[name].publish(tg)

        self.viz_pub.publish(self._viz())

    def _gate(self, pose, world_pts, ids, max_range, raw=False):
        """Apply range/FOV/facing for one robot; return a Markers (cm, body).

        `ids` is parallel to `world_pts` (stable ground-truth IDs); the surviving
        markers carry their IDs in msg.ids. When `raw`, inject position + normal
        noise and per-marker dropout to emulate the hardware detector.
        """
        rx, ry, theta_deg = pose
        th = math.radians(theta_deg) + self.cam_offset
        c, s = math.cos(th), math.sin(th)
        r2 = max_range * max_range

        msg = Markers()
        # Fresh stamp when built: clean fused output is always fresh; a genuine
        # raw fix mints a new stamp here, then tick() freezes it between fixes.
        msg.header.stamp = self.get_clock().now().to_msg()
        for idx, ((px, py), (nx, ny)) in enumerate(world_pts):
            dx, dy = px - rx, py - ry
            if dx * dx + dy * dy > r2:                      # range gate
                continue
            # Into body frame: body = R(-th) . (p_map - robot). x fwd, y left.
            bx = c * dx + s * dy
            by = -s * dx + c * dy
            if abs(math.atan2(by, bx)) > self.half_fov:     # FOV gate
                continue
            dist = math.hypot(dx, dy)
            if dist < 1e-6:
                continue
            if (nx * (-dx) + ny * (-dy)) / dist < self.cos_incidence:  # facing
                continue
            if raw and self.dropout_prob > 0.0 and \
                    random.random() < self.dropout_prob:    # detector dropout
                continue

            # Normal into body frame, then the published [right, fwd] convention
            # with forward component forced positive (aruco flip).
            nfx = c * nx + s * ny
            nfy = -s * nx + c * ny
            if raw and self.normal_noise_std > 0.0:
                nfx += random.gauss(0.0, self.normal_noise_std)
                nfy += random.gauss(0.0, self.normal_noise_std)
                mag = math.hypot(nfx, nfy)
                if mag > 1e-9:
                    nfx, nfy = nfx / mag, nfy / mag
            if nfx < 0.0:
                nfx, nfy = -nfx, -nfy

            pos = CoordXY()
            pos.x = bx * 100.0 + (self._noise() if raw else 0.0)
            pos.y = by * 100.0 + (self._noise() if raw else 0.0)
            nrm = CoordXY()
            nrm.x = -nfy          # X_cam (right) = -left
            nrm.y = nfx           # Z_cam (forward)
            msg.markers.append(pos)
            msg.markers_normals.append(nrm)
            if idx < len(ids):
                msg.ids.append(int(ids[idx]))
        return msg

    def _noise(self):
        if self.noise_std <= 0.0:
            return 0.0
        return random.gauss(0.0, self.noise_std)

    # -- visualization ----------------------------------------------------
    def _viz(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, o in enumerate(self.objects):
            base = i * 10
            outline = o.world_outline()

            # Boundary -- the solid object.
            line = self._base_marker(
                base + 0, stamp, Marker.LINE_STRIP, _OUTLINE_COLOR)
            line.scale = Vector3(x=0.03, y=0.0, z=0.0)
            for x, y in outline + outline[:1]:
                line.points.append(Point(x=x, y=y, z=0.0))
            arr.markers.append(line)

            # Landmarks: red dots + red normals.
            lm = o.world_landmarks()
            arr.markers.append(self._points_marker(
                base + 1, stamp, lm, _LANDMARK_COLOR, 0.03))
            arr.markers.append(self._normals_marker(
                base + 2, stamp, lm, _LANDMARK_COLOR))

            # Capture points: blue dots + blue normals.
            cp = o.world_captures()
            arr.markers.append(self._points_marker(
                base + 3, stamp, cp, _CAPTURE_COLOR, 0.06))
            arr.markers.append(self._normals_marker(
                base + 4, stamp, cp, _CAPTURE_COLOR))

            # CoM sphere.
            dot = self._base_marker(
                base + 5, stamp, Marker.SPHERE, _OUTLINE_COLOR)
            dot.scale = Vector3(x=0.06, y=0.06, z=0.06)
            dot.pose.position = Point(x=o.com[0], y=o.com[1], z=0.0)
            arr.markers.append(dot)

            # Heading arrow.
            head = self._base_marker(
                base + 6, stamp, Marker.ARROW, _OUTLINE_COLOR)
            head.scale = Vector3(x=0.02, y=0.05, z=0.0)
            head.points = [Point(x=o.com[0], y=o.com[1], z=0.0),
                           Point(x=o.com[0] + 0.25 * math.cos(o.yaw),
                                 y=o.com[1] + 0.25 * math.sin(o.yaw), z=0.0)]
            arr.markers.append(head)

            # Velocity arrow (only when moving).
            vel = self._base_marker(
                base + 7, stamp, Marker.ARROW, _OUTLINE_COLOR)
            vel.scale = Vector3(x=0.02, y=0.05, z=0.0)
            if math.hypot(o.vel[0], o.vel[1]) > 1e-3:
                vel.points = [Point(x=o.com[0], y=o.com[1], z=0.0),
                              Point(x=o.com[0] + o.vel[0],
                                    y=o.com[1] + o.vel[1], z=0.0)]
            else:
                vel.action = Marker.DELETE
            arr.markers.append(vel)
        return arr

    def _points_marker(self, mid, stamp, world_pts, color, diameter):
        m = self._base_marker(mid, stamp, Marker.SPHERE_LIST, color)
        m.scale = Vector3(x=diameter, y=diameter, z=diameter)
        for (x, y), _ in world_pts:
            m.points.append(Point(x=x, y=y, z=0.0))
        return m

    def _normals_marker(self, mid, stamp, world_pts, color):
        m = self._base_marker(mid, stamp, Marker.LINE_LIST, color)
        m.scale = Vector3(x=0.01, y=0.0, z=0.0)
        for (x, y), (nx, ny) in world_pts:
            m.points.append(Point(x=x, y=y, z=0.0))
            m.points.append(Point(x=x + _NORMAL_LEN * nx,
                                   y=y + _NORMAL_LEN * ny, z=0.0))
        return m

    def _base_marker(self, mid, stamp, mtype, color):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = "virtual_scene"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.9)
        m.pose.orientation.w = 1.0
        return m


def main(args=None):
    rclpy.init(args=args)
    node = VirtualScene()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
