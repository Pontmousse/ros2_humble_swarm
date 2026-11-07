"""Landmark/target memory estimator -- map-frame model fused from ArUco IDs.

Why this node exists
--------------------
On hardware the ArUco pipeline delivers only ~0.3 Hz of genuine detections, yet
the controllers consume the raw body-frame `Markers` directly. When markers leave
the FOV the reference vanishes, the freshness guard zeroes pointing torque, the
robot coasts, its own spin slings the markers further out of FOV, and a yaw limit
cycle builds. The virtual target never has this problem because `virtual_scene`
synthesises a steady reference every tick.

This node does the same for real perception: it keeps a MAP-FRAME memory of
landmarks keyed by ArUco ID, fuses each FRESH observation, and republishes the
whole remembered cloud at a steady rate, projected back into the body frame
through the clean `pose_kf` pose. Even with every marker out of FOV the controller
keeps a stable, smooth reference -- killing the limit cycle at its source.

Drop-in shape
-------------
Generic single stream: `markers_unf` (ID-tagged) + `pose` in, `markers` out --
exactly the `aruco_filter` contract, so it is instantiated twice via remappings
(landmark + target), each independently tuned. The modelling lives behind the
`estimator_type` seam (see estimators.py); v1 is per-ID cumulative averaging of a
static scene.

Frame convention (identical to virtual_scene._gate, so the round-trip is exact)
------------------------------------------------------------------------------
  * marker POSITION: body-frame centimetres, x=forward, y=left.
  * marker NORMAL: published [X_cam, Z_cam] = [right, forward], forward forced +.
  * pose: StatePos (x,y metres; theta DEGREES); th = radians(theta) + cam_offset.
  * body  = R(-th) . (p_map - robot)         (map -> body, forward direction)
  * p_map = robot + R(th) . body             (body -> map, the inverse)
We invert `_gate` on input (recover map position+normal from each observation) and
apply it forward on output (project the remembered map back to body), so what the
controllers receive is byte-compatible with the previous aruco_filter output.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from swarm_interfaces.msg import CoordXY, Markers, StatePos

from swarm_target.estimators import build_estimator


class LandmarkEstimator(Node):
    def __init__(self):
        super().__init__("landmark_estimator")

        # -- model + behaviour params ------------------------------------
        self.declare_parameter("estimator_type", "averaging")
        self.declare_parameter("publish_rate", 15.0)       # Hz, perception-free
        self.declare_parameter("min_observations", 2)      # confidence gate
        self.declare_parameter("forget_timeout", 0.0)      # s, 0 = keep forever
        self.declare_parameter("output_fov_gate", False)   # False -> full map out
        self.declare_parameter("output_max_range", 0.0)    # m, 0 = no range gate
        self.declare_parameter("half_fov_deg", 40.0)       # output FOV gate
        self.declare_parameter("max_incidence_deg", 70.0)  # output facing gate
        self.declare_parameter("camera_yaw_offset_deg", 0.0)
        self.declare_parameter("global_frame_id", "swarm_map")
        self.declare_parameter("publish_memory_viz", True)

        # -- marker-stream QoS (match the aruco_filter instance it replaces) --
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_reliability", "RELIABLE")
        self.declare_parameter("qos_history", "KEEP_LAST")

        et = str(self.get_parameter("estimator_type").value)
        rate = float(self.get_parameter("publish_rate").value)
        self.output_fov_gate = bool(self.get_parameter("output_fov_gate").value)
        out_range = float(self.get_parameter("output_max_range").value)
        self.output_r2 = out_range * out_range          # 0 -> gate disabled
        self.half_fov = math.radians(
            float(self.get_parameter("half_fov_deg").value))
        self.cos_incidence = math.cos(math.radians(
            float(self.get_parameter("max_incidence_deg").value)))
        self.cam_offset = math.radians(
            float(self.get_parameter("camera_yaw_offset_deg").value))
        self.frame_id = str(self.get_parameter("global_frame_id").value)
        self.publish_memory_viz = bool(
            self.get_parameter("publish_memory_viz").value)

        self.estimator = build_estimator(
            et,
            min_observations=int(self.get_parameter("min_observations").value),
            forget_timeout=float(self.get_parameter("forget_timeout").value),
        )

        marker_qos = self._qos_from_params()
        # pose is published BEST_EFFORT (controllers subscribe the same way).
        pose_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pose = None                 # (x, y, theta_deg); None until first pose
        self._last_stamp = None          # (sec, nsec) of the last FRESH detection

        self.sub_markers = self.create_subscription(
            Markers, "markers_unf", self._on_markers, marker_qos)
        self.sub_pose = self.create_subscription(
            StatePos, "pose", self._on_pose, pose_qos)

        self.pub = self.create_publisher(Markers, "markers", marker_qos)
        self.viz_pub = self.create_publisher(
            MarkerArray, "landmark_memory/markers", 1)

        self.last_time = self.get_clock().now()
        self.create_timer(1.0 / max(rate, 1e-6), self._tick)

        self.get_logger().info(
            f"landmark_estimator: type={et!r}, publish_rate={rate:.1f} Hz, "
            f"min_obs={self.estimator.min_observations if hasattr(self.estimator, 'min_observations') else '?'}, "
            f"output_fov_gate={self.output_fov_gate}, frame={self.frame_id!r}.")

    # -- QoS -------------------------------------------------------------
    def _qos_from_params(self):
        depth = int(self.get_parameter("qos_depth").value)
        rel = str(self.get_parameter("qos_reliability").value)
        hist = str(self.get_parameter("qos_history").value)
        if rel not in ("RELIABLE", "BEST_EFFORT"):
            raise ValueError(f"Invalid qos_reliability: {rel}")
        if hist not in ("KEEP_LAST", "KEEP_ALL"):
            raise ValueError(f"Invalid qos_history: {hist}")
        return QoSProfile(
            depth=depth,
            reliability=(ReliabilityPolicy.RELIABLE if rel == "RELIABLE"
                         else ReliabilityPolicy.BEST_EFFORT),
            history=(HistoryPolicy.KEEP_LAST if hist == "KEEP_LAST"
                     else HistoryPolicy.KEEP_ALL),
        )

    # -- callbacks -------------------------------------------------------
    def _on_pose(self, msg: StatePos):
        self.pose = (msg.x, msg.y, msg.theta)

    def _on_markers(self, msg: Markers):
        """Fuse ONLY genuinely fresh detections.

        The detector republishes the held frame at ~100 Hz with a FROZEN stamp
        between real fixes; fusing those would over-weight a single observation.
        We key on header.stamp changing, mirroring pose_kf's Marvelmind gate.
        """
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self._last_stamp:
            return                        # frozen republish -> not a new fix
        self._last_stamp = stamp

        if self.pose is None:
            return                        # need a pose to lift body -> map
        if not msg.markers:
            return                        # fresh-but-empty: nothing to fuse

        ids = list(msg.ids)
        if len(ids) != len(msg.markers):
            self.get_logger().warn(
                f"markers_unf has {len(msg.markers)} markers but {len(ids)} "
                "ids; cannot associate -- dropping frame.",
                throttle_duration_sec=5.0)
            return

        rx, ry, theta_deg = self.pose
        th = math.radians(theta_deg) + self.cam_offset
        c, s = math.cos(th), math.sin(th)
        t = self.get_clock().now().nanoseconds * 1e-9

        normals = msg.markers_normals
        have_normals = len(normals) == len(msg.markers)
        for i, m in enumerate(msg.markers):
            # Invert _gate position: body-cm -> body-m -> map.
            bx = m.x / 100.0
            by = m.y / 100.0
            px = rx + (c * bx - s * by)
            py = ry + (s * bx + c * by)

            # Invert _gate normal: [right, fwd] -> body (fwd,left) -> map.
            if have_normals:
                nrm = normals[i]
                nfx = nrm.y            # forward   (published nrm.y = nfx)
                nfy = -nrm.x           # left      (published nrm.x = -nfy)
            else:
                nfx, nfy = 1.0, 0.0    # unknown normal -> point along +forward
            nx = c * nfx - s * nfy
            ny = s * nfx + c * nfy

            self.estimator.observe(ids[i], (px, py), (nx, ny), t)

    # -- steady output ---------------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        self.estimator.predict(dt)

        if self.pose is None:
            return                        # cannot project without a pose yet

        estimates = self.estimator.landmarks()

        rx, ry, theta_deg = self.pose
        th = math.radians(theta_deg) + self.cam_offset
        c, s = math.cos(th), math.sin(th)

        out = Markers()
        # Always-fresh stamp: the whole point is to decouple the controller's
        # freshness guard from the slow detector. A remembered landmark is a
        # live, confident estimate at THIS instant.
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.frame_id
        out.markers = []
        out.markers_normals = []
        out.ids = []

        for est in estimates:
            px, py = float(est.p[0]), float(est.p[1])
            nx, ny = float(est.n[0]), float(est.n[1])

            dx, dy = px - rx, py - ry
            if self.output_fov_gate:
                if self.output_r2 > 0.0 and (dx * dx + dy * dy) > self.output_r2:
                    continue
            # Forward _gate: map -> body (x fwd, y left).
            bx = c * dx + s * dy
            by = -s * dx + c * dy
            if self.output_fov_gate:
                if abs(math.atan2(by, bx)) > self.half_fov:
                    continue
                dist = math.hypot(dx, dy)
                if dist > 1e-6 and \
                        (nx * (-dx) + ny * (-dy)) / dist < self.cos_incidence:
                    continue

            # Normal -> body, then the published [right, fwd] convention.
            nfx = c * nx + s * ny
            nfy = -s * nx + c * ny
            if nfx < 0.0:
                nfx, nfy = -nfx, -nfy

            pos = CoordXY()
            pos.x = bx * 100.0
            pos.y = by * 100.0
            nrm = CoordXY()
            nrm.x = -nfy          # X_cam (right) = -left
            nrm.y = nfx           # Z_cam (forward)
            out.markers.append(pos)
            out.markers_normals.append(nrm)
            out.ids.append(int(est.id))

        self.pub.publish(out)

        if self.publish_memory_viz:
            self.viz_pub.publish(self._memory_viz(estimates, now))

    # -- debug visualization (map frame) ---------------------------------
    def _memory_viz(self, estimates, now):
        arr = MarkerArray()
        stamp = now.to_msg()

        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for k, est in enumerate(estimates):
            px, py = float(est.p[0]), float(est.p[1])
            nx, ny = float(est.n[0]), float(est.n[1])
            base = k * 3

            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = stamp
            sphere.ns = "landmark_memory"
            sphere.id = base
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = Point(x=px, y=py, z=0.0)
            sphere.pose.orientation.w = 1.0
            sphere.scale = Vector3(x=0.08, y=0.08, z=0.08)
            a = 0.4 + 0.5 * float(min(1.0, est.confidence))
            sphere.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=a)
            arr.markers.append(sphere)

            normal = Marker()
            normal.header.frame_id = self.frame_id
            normal.header.stamp = stamp
            normal.ns = "landmark_memory"
            normal.id = base + 1
            normal.type = Marker.LINE_LIST
            normal.action = Marker.ADD
            normal.scale.x = 0.01
            normal.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.9)
            normal.points.append(Point(x=px, y=py, z=0.0))
            normal.points.append(
                Point(x=px + 0.08 * nx, y=py + 0.08 * ny, z=0.0))
            arr.markers.append(normal)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "landmark_memory"
            label.id = base + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(x=px, y=py, z=0.12)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.06
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            label.text = f"{est.id} (n={est.count})"
            arr.markers.append(label)

        return arr

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
