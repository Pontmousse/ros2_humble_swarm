"""Drive the three-mode mission state machine: SEARCH -> ENCAPSULATION -> CAPTURE.

Each mode is a different wrench source on the shared virtual spacecraft; the
physical robot tracks the spacecraft the same way in every mode (pidff +
velocity_smoother), so switching modes is nothing more than toggling which
wrench sources are enabled. There is no cmd_vel handoff, no smoother pause, and
no spacecraft reset: every layer publishes into the summed spacecraft input and
a disabled layer simply publishes zero (gating by zero), so a transition is two
SetBool calls and nothing else.

Default transition policy is detection-driven, with the usual hysteresis:

    SEARCH        --stable landmarks--> ENCAPSULATION
    SEARCH/ENCAP  --stable targets----> CAPTURE          (targets always win)
    ENCAP         --landmarks lost----> SEARCH
    CAPTURE       --target lost--------> ENCAP/SEARCH     (unless latch_capture)

EXTENSION POINT -- the policy is deliberately isolated so it can be replaced
later by a manual trigger or swarm consensus without touching the detection
plumbing or the actuation:

  * `decide()` is a pure function of (current mode, detection gates, override).
    Rewrite its body, or
  * publish a mode name on the `mode_cmd` topic (std_msgs/String, latched). A
    non-empty value pins the mode (manual trigger / consensus output); an empty
    string hands control back to the detection policy.

So "collective capture by consensus" later is: a consensus node agrees on a
mode and publishes it to every robot's `mode_cmd`. Nothing else here changes.

Actuation (2026-10): each mode maps to exactly one SetBool wrench gate --
  SEARCH        -> `search/enable`        (bounding_box_search -> spacecraft_wrench)
  ENCAPSULATION -> `encapsulation/enable` (position_guidance3  -> flocking_wrench)
  CAPTURE       -> `toggle_controller`    (cp_controller       -> capture_wrench)
  DOCK          -> `toggle_dock`          (cp_controller       -> freeze, zero wrench)
All four servers exist. `avoidance_wrench` is NOT gated here -- it is an
always-on peer source summed by the spacecraft in every mode, so inter-robot
avoidance never depends on which mode is active. Calls are fire-and-forget and
only warn if a server is briefly absent at startup.

DOCK is entered from CAPTURE once `contact_detected` (std_msgs/Bool) goes
True. That topic is a PLANT-layer signal -- published by fake_plant.py under
VIRTUAL_PLANT, or by a future physical end-of-course-sensor node on real
hardware -- so this supervisor and cp_controller never need to know which one
is active. DOCK is terminal for now: only a mode_cmd override leaves it.
"""

from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
from std_msgs.msg import Bool, String
from swarm_interfaces.msg import Markers


class Mode(Enum):
    SEARCH = "search"
    ENCAPSULATION = "encapsulation"
    CAPTURE = "capture"
    DOCK = "dock"


# Accept a few aliases on mode_cmd so a human override is forgiving.
_MODE_ALIASES = {
    "": None,  # empty -> hand back to the detection policy
    "auto": None,
    "search": Mode.SEARCH,
    "encapsulation": Mode.ENCAPSULATION,
    "ep": Mode.ENCAPSULATION,
    "capture": Mode.CAPTURE,
    "cp": Mode.CAPTURE,
    "dock": Mode.DOCK,
}


class DetectionGate:
    """Detection hysteresis for one Markers topic.

    Accumulates consecutive non-empty frames (hit_streak) while tolerating a
    short run of empty frames (miss_tolerance), so low-fps detection flicker
    does not reset the streak. `stable` latches once detect_frames is reached;
    `age` reports seconds since the last non-empty frame for loss decisions.
    """

    def __init__(self, detect_frames: int, miss_tolerance: int):
        self.detect_frames = detect_frames
        self.miss_tolerance = miss_tolerance
        self.hit_streak = 0
        self.miss_streak = 0
        self.last_seen = None  # rclpy Time of the last non-empty frame

    def update(self, present: bool, now):
        if present:
            self.last_seen = now
            self.miss_streak = 0
            self.hit_streak += 1
        else:
            self.miss_streak += 1
            if self.miss_streak > self.miss_tolerance:
                self.hit_streak = 0

    @property
    def stable(self) -> bool:
        return self.hit_streak >= self.detect_frames

    def age(self, now) -> float:
        if self.last_seen is None:
            return float("inf")
        return (now - self.last_seen).nanoseconds * 1e-9


class ModeSupervisor(Node):
    """Three-mode mission supervisor with a pluggable transition policy."""

    def __init__(self):
        super().__init__("mode_supervisor")

        # Detection hysteresis (shared by the landmark and target gates).
        self.declare_parameter("detect_frames", 5)
        # Seconds a signal may stay lost before a backward transition fires.
        self.declare_parameter("lost_timeout", 2.0)
        # Consecutive EMPTY frames tolerated before an entry streak resets.
        self.declare_parameter("miss_tolerance", 3)
        # Sticky capture: once latched into CAPTURE, never auto-fall-back on
        # target loss. Kills CAPTURE<->SEARCH oscillation from low-fps flicker.
        self.declare_parameter("latch_capture", True)
        # When False, detection never auto-promotes to CAPTURE; capture is reached
        # only via a mode_cmd override (manual go-signal now, swarm consensus later).
        self.declare_parameter("auto_capture", True)
        # Mode the node assumes it starts in. The launch is expected to bring up
        # search-enabled / capture-disabled, so SEARCH matches reality without
        # the supervisor forcing any service call at startup.
        self.declare_parameter("initial_mode", "search")
        self.declare_parameter("qos_reliability", "BEST_EFFORT")

        detect_frames = int(self.get_parameter("detect_frames").value)
        miss_tolerance = int(self.get_parameter("miss_tolerance").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout").value)
        self.latch_capture = bool(self.get_parameter("latch_capture").value)
        self.auto_capture = bool(self.get_parameter("auto_capture").value)
        reliability = str(self.get_parameter("qos_reliability").value)
        self.mode = _MODE_ALIASES.get(
            str(self.get_parameter("initial_mode").value).lower()) or Mode.SEARCH

        qos = QoSProfile(
            depth=10,
            reliability=(ReliabilityPolicy.RELIABLE
                         if reliability == "RELIABLE"
                         else ReliabilityPolicy.BEST_EFFORT),
            history=HistoryPolicy.KEEP_LAST,
        )

        # Detection gates. One per signal that can drive a transition.
        self.landmarks = DetectionGate(detect_frames, miss_tolerance)
        self.targets = DetectionGate(detect_frames, miss_tolerance)

        # Plant-layer contact signal (see module docstring). Plain bool, not
        # a DetectionGate -- the debouncing already happens at the source
        # (fake_plant.py / the future hardware node), and DOCK's own
        # stickiness in decide() is what makes the transition terminal.
        self.contact = False

        # Manual / consensus override. None -> detection policy is in charge.
        self.mode_override = None

        # Service clients, one SetBool gate per mode. Relative names resolve
        # under the launch namespace into /RMx/<name>.
        #  - search/enable          : bounding_box_search wrench (SEARCH)
        #  - encapsulation/enable   : flocking wrench           (ENCAPSULATION)
        #  - toggle_controller      : cp_controller capture wrench (CAPTURE)
        #  - toggle_dock            : cp_controller freeze         (DOCK)
        self.search_client = self.create_client(SetBool, "search/enable")
        self.encap_client = self.create_client(SetBool, "encapsulation/enable")
        self.capture_client = self.create_client(SetBool, "toggle_controller")
        self.dock_client = self.create_client(SetBool, "toggle_dock")
        # One client per mode, keyed so apply_mode is a two-line toggle.
        self.mode_clients = {
            Mode.SEARCH: self.search_client,
            Mode.ENCAPSULATION: self.encap_client,
            Mode.CAPTURE: self.capture_client,
            Mode.DOCK: self.dock_client,
        }

        # Detection inputs.
        self.create_subscription(Markers, "landmarks", self.on_landmarks, qos)
        self.create_subscription(Markers, "targets", self.on_targets, qos)
        self.create_subscription(Bool, "contact_detected", self.on_contact, qos)

        # Mode override channel (manual trigger / consensus). Latched so a
        # single `ros2 topic pub --once` (or a late consensus message) sticks.
        override_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            String, "mode_cmd", self.on_mode_cmd, override_qos)
        # Absolute twin: one `/mode_cmd` publish triggers every robot at once
        # (manual go-signal for simultaneous capture), same callback/QoS.
        self.create_subscription(
            String, "/mode_cmd", self.on_mode_cmd, override_qos)

        # Evaluate the policy at 5 Hz. Detection gates are updated in the topic
        # callbacks; this timer only decides + actuates.
        self.create_timer(0.2, self.tick)

        self.get_logger().info(
            f"Mode supervisor ready in {self.mode.value.upper()} "
            "(publish mode_cmd to override; empty string = auto).")

    # ------------------------------ inputs ---------------------------------
    def on_landmarks(self, msg: Markers):
        self.landmarks.update(len(msg.markers) > 0, self.get_clock().now())

    def on_targets(self, msg: Markers):
        self.targets.update(len(msg.markers) > 0, self.get_clock().now())

    def on_contact(self, msg: Bool):
        self.contact = bool(msg.data)

    def on_mode_cmd(self, msg: String):
        key = msg.data.strip().lower()
        if key not in _MODE_ALIASES:
            self.get_logger().warn(
                f"ignoring mode_cmd {msg.data!r}; use one of "
                f"{sorted(k for k in _MODE_ALIASES if k)} or '' for auto.")
            return
        self.mode_override = _MODE_ALIASES[key]
        where = (self.mode_override.value if self.mode_override
                 else "auto (detection)")
        self.get_logger().info(f"mode_cmd -> override = {where}")

    # ---------------------------- decision ---------------------------------
    def decide(self) -> Mode:
        """Return the desired mode. THE pluggable policy (see module docstring).

        Pure function of current mode, the detection gates, and the override.
        Replace the body, or drive `mode_override` via `mode_cmd`, to switch to
        manual / consensus triggering without touching anything below.
        """
        if self.mode_override is not None:
            return self.mode_override

        # DOCK is terminal for now -- checked before everything else so a
        # still-visible/stable target can never pull a docked robot back into
        # CAPTURE. Only a mode_cmd override leaves DOCK.
        if self.mode == Mode.DOCK:
            return Mode.DOCK

        # Contact while capturing -> DOCK, above the auto_capture short-circuit
        # below so a still-stable target can't keep a touching robot in CAPTURE.
        if self.mode == Mode.CAPTURE and self.contact:
            return Mode.DOCK

        now = self.get_clock().now()

        # A stable target wins -- go straight to CAPTURE from any mode. Disabled
        # by auto_capture=False, which reserves CAPTURE for the mode_cmd override.
        if self.auto_capture and self.targets.stable:
            return Mode.CAPTURE

        if self.mode == Mode.SEARCH:
            if self.landmarks.stable:
                return Mode.ENCAPSULATION
            return Mode.SEARCH

        if self.mode == Mode.ENCAPSULATION:
            # Fall back to SEARCH only once landmarks are gone past the timeout.
            if self.landmarks.age(now) > self.lost_timeout:
                return Mode.SEARCH
            return Mode.ENCAPSULATION

        # CAPTURE (contact -> DOCK is handled at the top, above the override).
        if self.latch_capture:
            return Mode.CAPTURE
        if self.targets.age(now) > self.lost_timeout:
            # Prefer ENCAP if landmarks are still around, else all the way back.
            if self.landmarks.age(now) <= self.lost_timeout:
                return Mode.ENCAPSULATION
            return Mode.SEARCH
        return Mode.CAPTURE

    def tick(self):
        desired = self.decide()
        if desired != self.mode:
            self.apply_mode(desired)

    # ---------------------------- actuation --------------------------------
    def apply_mode(self, new: Mode):
        """Move from self.mode to `new` by toggling the two wrench gates.

        Every mode is one SetBool-gated wrench source and the tracker follows
        the summed spacecraft identically in all modes, so a transition is just:
        disable the departing layer, enable the arriving one. Disable first so
        two guidance layers never both claim the full envelope for an instant.
        """
        old = self.mode
        self.get_logger().info(f"MODE {old.value} -> {new.value}")
        self.call(self.mode_clients[old], False)
        self.call(self.mode_clients[new], True)
        self.mode = new

    def call(self, client, state: bool):
        """Fire-and-forget SetBool request; warn if the service is absent."""
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{client.srv_name} not available.")
            return
        request = SetBool.Request()
        request.data = state
        client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = ModeSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
