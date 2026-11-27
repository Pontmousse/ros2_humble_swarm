"""perception: localization KF + landmark/target memories + the v2 driver inputs, in one process.

Replaces pose_kf, landmark_estimator x2, gnss_bridge and aruco_bridge (nodes-to-functions Phase 2). The KF
(PoseKalmanFilterNode, inherited unchanged) publishes pose, localization/odom and TF on its 20 Hz tick; right
after, both memories are projected through THAT pose and published as landmarks/targets with the same stamp. So
the controllers keep a fresh, steady marker cloud through camera stalls (the memory), and it never comes from a
stale pose: when the KF withholds its output (not initialised, dead odom) the markers are withheld too.

This file only wires parameters, topics and timers; the logic lives in marker_memory, pose_history,
swarm_aruco.aruco_convert and swarm_marvelmind.gnss_convert.

position_input  v2      gnss/position (drivers container); with debug_topics, republished as mm_pos_unf.
                legacy  mm_pos_unf (fake_plant, VIRTUAL_PLANT).
marker_input    v2      aruco/landmarks + aruco/markers (drivers container). Republishes targets_unf
                        (readiness_consensus raw liveness) and, with debug_topics, landmarks_unf.
                legacy  landmarks_unf + targets_unf (virtual_scene, VIRTUAL_TARGET; also HIL with real robots).
"""
import rclpy
from geometry_msgs.msg import PointStamped
from marvelmind_ros2_msgs.msg import HedgePositionAddressed
from rclpy.executors import ExternalShutdownException
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from swarm_aruco.aruco_convert import to_markers
from swarm_filter.pose_kf import PoseKalmanFilterNode
from swarm_filter.pose_publisher import degrees
from swarm_interfaces.msg import ArucoMarkerArray, Markers
from swarm_marvelmind.gnss_convert import to_hedge
from visualization_msgs.msg import MarkerArray

from swarm_target.marker_memory import MarkerMemory, memory_viz
from swarm_target.pose_history import PoseHistory

# set name, raw topic (legacy input / v2 republish), drivers topic (v2 input), memory viz topic
SETS = (("landmarks", "landmarks_unf", "aruco/landmarks", "landmark_memory/markers"),
        ("targets", "targets_unf", "aruco/markers", "target_memory/markers"))
MEMORY_PARAMS = dict(estimator_type="averaging", min_observations=2, forget_timeout=0.0, camera_yaw_offset_deg=0.0,
                     output_fov_gate=False, output_max_range=0.0, half_fov_deg=40.0, max_incidence_deg=70.0)


class PerceptionNode(PoseKalmanFilterNode):
    def __init__(self):
        super().__init__("perception")
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        gnss_v2 = str(p("position_input", "v2")) == "v2"
        markers_v2 = str(p("marker_input", "v2")) == "v2"
        debug = bool(p("debug_topics", True))
        beacon = int(p("beacon", 0))
        self.lift_at_stamp = bool(p("lift_at_stamp", False))
        viz_rate = float(p("memory_viz_rate", 0.0))
        self.viz_period = 1.0 / viz_rate if viz_rate > 0.0 else 0.0
        qos = self._qos_from_parameters()                    # same QoS as pose (BEST_EFFORT, depth 1 in the launch)
        drivers_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        self.pose_now, self.last_t, self.last_viz = None, None, 0.0
        self.history = PoseHistory()
        self.sets = []
        for name, unf, aruco, viz in SETS:
            memory = MarkerMemory(**{k: type(d)(p(f"{name}.{k}", d)) for k, d in MEMORY_PARAMS.items()})
            self.sets.append((name, memory, self.create_publisher(Markers, name, qos),
                              self.create_publisher(MarkerArray, viz, 1) if self.viz_period else None))
            if not markers_v2:
                self.create_subscription(Markers, unf, lambda m, mem=memory: self._on_markers(mem, m), qos)
                continue
            scale = tuple(p(f"{name}.scale", [1.0, 1.0]))
            max_cm = 100.0 * float(p(f"{name}.max_range", 0.0))
            raw_pub = self.create_publisher(Markers, unf, drivers_qos) if debug or name == "targets" else None
            self.create_subscription(ArucoMarkerArray, aruco, lambda m, mem=memory, s=scale, r=max_cm, pub=raw_pub:
                                     self._on_aruco(mem, to_markers(m, s, r), pub), drivers_qos)
        if gnss_v2:
            # The KF's own mm_pos_unf subscription also hears the debug republish; its (x, y, timestamp_ms)
            # fresh gate drops that echo, so each fix is fused once.
            mm_pub = self.create_publisher(HedgePositionAddressed, "mm_pos_unf", drivers_qos) if debug else None
            self.create_subscription(PointStamped, "gnss/position",
                                     lambda m: self._on_gnss(to_hedge(m, beacon), mm_pub), drivers_qos)
        self.get_logger().info(f"perception: position_input={'v2' if gnss_v2 else 'legacy'}, "
                               f"marker_input={'v2' if markers_v2 else 'legacy'}, debug_topics={debug}, "
                               f"lift_at_stamp={self.lift_at_stamp}, memory_viz_rate={viz_rate}")

    # -- inputs ----------------------------------------------------------------
    def _on_gnss(self, hedge, pub):
        if pub:
            pub.publish(hedge)
        self._on_marvelmind(hedge)

    def _on_aruco(self, memory, markers, pub):
        if pub:
            pub.publish(markers)
        self._on_markers(memory, markers)

    def _on_markers(self, memory, msg):
        pose = self.pose_now
        if self.lift_at_stamp and pose is not None:
            pose = self.history.at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, pose)
        if memory.observe(msg, pose, self._now_ns() * 1e-9) < 0:
            self.get_logger().warn(f"{len(msg.markers)} markers but {len(msg.ids)} ids; cannot associate, "
                                   "dropping frame.", throttle_duration_sec=5.0)

    # -- output: the KF tick, then both memories through the pose just published -------------------------
    def _publish(self, stamp):
        super()._publish(stamp)
        t = stamp.sec + stamp.nanosec * 1e-9
        self.pose_now = (float(self.xp[0]), float(self.xp[1]), degrees(self.theta))
        self.history.add(t, self.pose_now)
        dt, self.last_t = (t - self.last_t if self.last_t is not None else 0.0), t
        viz = self.viz_period and t - self.last_viz >= self.viz_period
        for name, memory, pub, viz_pub in self.sets:
            memory.predict(dt)
            estimates = memory.estimates()
            out = memory.project(self.pose_now, estimates)
            out.header.stamp, out.header.frame_id = stamp, self.global_frame_id
            pub.publish(out)
            if viz:
                viz_pub.publish(memory_viz(estimates, stamp, self.global_frame_id, f"{name[:-1]}_memory"))
        if viz:
            self.last_viz = t


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
