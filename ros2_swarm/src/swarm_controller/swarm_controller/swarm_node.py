"""swarm: AP auction + readiness consensus + mode supervisor + contact source + run_config, in one process.

Replaces ap_auction, readiness_consensus, mode_supervisor, contact_switch / geometric_contact and
run_config_snapshot (nodes-to-functions Phase 5). One 5 Hz tick runs auction -> consensus -> supervisor, so the
consensus uses this tick's assignment and the supervisor acts in the tick capture_sync fires. In-process fan-outs
replace the internal hops (assigned_ap / assignment_locked -> consensus, capture_sync and contact_detected ->
supervisor); those topics are still published for guidance, peers and bags. This file only wires parameters,
topics and timers; the logic lives in ap_auction, readiness_consensus, mode_supervisor, geometric_contact,
swarm_magnet.contact_switch and run_config.
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String
from swarm_interfaces.msg import ApBids, Markers, NeighbourStates, Readiness

from .ap_auction import Auction
from .geometric_contact import GeometricContact
from .mode_supervisor import ModeSupervisor
from .readiness_consensus import ReadinessConsensus
from .run_config import snapshot

GLOBAL = ('robot_id',)   # shared by auction and consensus; everything else is <class prefix>.<name>


class Fanout:
    def __init__(self, *sinks): self.sinks = list(sinks)
    def publish(self, msg): [sink(msg) for sink in self.sinks]


class SwarmNode(Node):
    def __init__(self):
        super().__init__('swarm')
        values = {}
        param = lambda n, d: values[n] if n in values else values.setdefault(n, self.declare_parameter(n, d).value)  # noqa: E731
        law_p = lambda prefix: (lambda n, d: param(n if n in GLOBAL else f'{prefix}.{n}', d))  # noqa: E731
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        pub = lambda t, topic, q: self.create_publisher(t, topic, q)  # noqa: E731

        self.supervisor = s = ModeSupervisor(self, law_p('supervisor'), pub(String, 'current_mode', latched))
        self.consensus = c = ReadinessConsensus(self, law_p('consensus'), pub(Readiness, '/readiness', best_effort),
                                                Fanout(pub(Bool, 'capture_sync', latched).publish, s.on_capture_sync))
        self.auction = a = Auction(self, law_p('auction'), pub(ApBids, '/ap_bids', best_effort),
                                   Fanout(pub(Int32, 'assigned_ap', reliable).publish, c.on_assigned),
                                   Fanout(pub(Bool, 'assignment_locked', reliable).publish, c.on_locked))

        # Contact source: gpio (Pi limit switch), geometric (SIL, from targets) or none.
        source, contact_p = str(param('contact.source', 'none')), law_p('contact')
        contact_pub = Fanout(pub(Bool, 'contact_detected', reliable).publish, s.on_contact)
        self.contact, on_targets = None, [a.on_targets, c.on_targets, s.on_targets]
        if source == 'gpio':
            from swarm_magnet.contact_switch import ContactSwitch   # gpiod only where there is GPIO
            self.contact = ContactSwitch(self.get_logger, contact_p, contact_pub)
            if self.contact.request is not None:
                self.create_timer(self.contact.period, self.contact.tick)
        elif source == 'geometric':
            self.contact = GeometricContact(contact_p, contact_pub)
            on_targets.append(self.contact._on_targets)
            self.create_timer(self.contact.period, self.contact.tick)
        elif source != 'none':
            raise ValueError(f"contact.source must be gpio, geometric or none, not {source!r}")

        self.create_subscription(Markers, 'targets', lambda m: [cb(m) for cb in on_targets], best_effort)
        self.create_subscription(Markers, 'landmarks', s.on_landmarks, best_effort)
        self.create_subscription(Markers, 'targets_unf', c.on_targets_unf, best_effort)
        self.create_subscription(Odometry, 'localization/odom', a.on_odom, best_effort)
        self.create_subscription(NeighbourStates, 'comms/neighbours',
                                 lambda m: (a.on_neighbours(m), c.on_neighbours(m)), best_effort)
        self.create_subscription(ApBids, '/ap_bids', a.on_peer_bids, best_effort)
        self.create_subscription(Readiness, '/readiness', c.on_peer, best_effort)
        # Other contact publishers too (the Pi's standalone switch in a centralized HIL run); contact is an OR latch.
        self.create_subscription(Bool, 'contact_detected', s.on_contact, best_effort)
        # Mode override (manual trigger / consensus), latched; /mode_cmd triggers every robot at once.
        for topic in ('mode_cmd', '/mode_cmd'):
            self.create_subscription(String, topic, s.on_mode_cmd, latched)

        # Latched once: the configs and env this run was launched with (self-describing bags).
        files = [f for f in param('run_config.config_files', ['']) if f]
        if files:
            msg, missing = snapshot(files, [e for e in param('run_config.env_vars', ['']) if e],
                                    param('run_config.repo_path', ''), param('run_config.source_launch', ''),
                                    [r for r in param('run_config.robot_names', ['']) if r])
            if missing:
                self.get_logger().warning(f"run_config snapshot could not read {len(missing)} file(s): " + ", ".join(missing))
            self.run_config_pub = pub(String, 'run_config', latched)
            self.run_config_pub.publish(msg)

        self.create_timer(c.dt, self.tick)
        self.get_logger().info(f'swarm: robot_id={a.robot_id}, {1.0 / c.dt:.0f} Hz, contact={source}')

    def tick(self):
        self.auction.round()
        self.consensus.round()
        self.supervisor.tick()

    def destroy_node(self):
        if hasattr(self.contact, 'release'):
            self.contact.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SwarmNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
