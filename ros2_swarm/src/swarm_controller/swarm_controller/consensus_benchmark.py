"""Seeded software coordination simulation; no DDS, sockets, or physical plant."""
import argparse
import ast
import csv
from collections import Counter, defaultdict
import hashlib
import heapq
import inspect
import json
import math
from pathlib import Path
import random
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch

import yaml
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Int32
from swarm_interfaces.msg import CoordXY, Markers, NeighbourState, NeighbourStates, Readiness

from swarm_comms import packet, table
from swarm_controller import mode_supervisor, readiness_consensus_node

NS = 1_000_000_000
EPOCH = 100 * NS
TOPOLOGIES = ('complete', 'chain', 'ring', 'star', 'random')
PROFILES = ('reference', 'nominal', 'latency', 'loss', 'perception', 'frozen',
            'clock', 'never_ready', 'partition')
NORMAL = ('reference', 'nominal')
OUTCOMES = ('synchronized_capture', 'unsynchronized_capture', 'partial_capture', 'no_capture')
DEFAULT_MODEL = dict(availability_max=4.0, raw_hz=10.0, estimated_hz=15.0,
                     assignment_hz=5.0, distance_m=0.6, delay_min_ms=53.6,
                     delay_max_ms=103.6, stress_delay_max_ms=600.0,
                     loss_max=0.2, fault_start_max=8.0, perception_min=0.2,
                     perception_max=1.5, clock_offset_ms=50.0,
                     partition_min=1.0, partition_max=3.0, local_delay_ms=0.0)


def ns(seconds):
    return round(seconds * NS)


def stream(*identity):
    digest = hashlib.sha256(json.dumps(identity, separators=(',', ':')).encode()).digest()
    return random.Random(int.from_bytes(digest, 'big'))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def declared_defaults(path):
    result = {}
    for call in ast.walk(ast.parse(Path(path).read_text())):
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute) \
                and call.func.attr == 'declare_parameter' and len(call.args) >= 2:
            value = call.args[1]
            if isinstance(value, ast.Name) and value.id in ('DEFAULT_BROADCAST', 'DEFAULT_PORT'):
                value = ast.Constant(getattr(packet, value.id))
            result[ast.literal_eval(call.args[0])] = ast.literal_eval(value)
    return result


def launch_parameters(path, defaults, spacecraft):
    """Read literal parameter blocks without importing or executing a launch file."""
    found = {}
    for call in ast.walk(ast.parse(Path(path).read_text())):
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name) or call.func.id != 'Node':
            continue
        kw = {item.arg: item.value for item in call.keywords}
        executable = kw.get('executable')
        if not isinstance(executable, ast.Constant) or executable.value not in defaults:
            continue
        name = executable.value
        if name in found:
            raise ValueError(f'{path}: multiple {name} blocks')
        values = dict(defaults[name])
        params = kw.get('parameters')
        if not isinstance(params, ast.List):
            raise ValueError(f'{path}: unsupported parameters for {name}')
        for block in params.elts:
            if isinstance(block, ast.Name) and block.id == 'spacecraft_params':
                for selector, content in spacecraft.items():
                    applicable = selector in ('/**', f'/**/{name}', f'/{name}')
                    overrides = content.get('ros__parameters', {}) or {}
                    if not applicable and any(key in values for key in overrides):
                        # Other named nodes are harmless; per-robot selectors need explicit support.
                        if selector.endswith('/' + name):
                            raise ValueError(f'unsupported per-robot YAML selector: {selector}')
                    if applicable:
                        values.update({k: v for k, v in overrides.items() if k in values})
                continue
            if not isinstance(block, ast.Dict):
                raise ValueError(f'{path}: unsupported parameter block for {name}')
            for key, value in zip(block.keys, block.values):
                if key is None:
                    if not isinstance(value, ast.Call) or not isinstance(value.func, ast.Name) \
                            or value.func.id != 'qos_parameters':
                        raise ValueError(f'{path}: unsupported parameter expansion')
                    continue  # These nodes construct their own QoS; overrides are not read by them.
                key = ast.literal_eval(key)
                if key in ('robot_id', 'broadcast_address', 'bind_device'):
                    continue  # Identity is assigned per agent; no real network is opened.
                if key not in values:
                    raise ValueError(f'{path}: unrecognized {name} parameter {key}')
                try:
                    values[key] = ast.literal_eval(value)
                except (ValueError, TypeError) as exc:
                    raise ValueError(f'{path}: nonliteral {name}.{key}') from exc
        for key in ('robot_id', 'broadcast_address', 'bind_device'):
            values.pop(key, None)
        found[name] = values
    if set(found) != set(defaults):
        raise ValueError(f'{path}: missing launch blocks: {set(defaults) - set(found)}')
    return found


def load_config(root):
    root = Path(root).resolve()
    src = root / 'ros2_swarm/src'
    files = {
        'consensus': src / 'swarm_controller/swarm_controller/readiness_consensus_node.py',
        'supervisor': src / 'swarm_controller/swarm_controller/mode_supervisor.py',
        'comms': src / 'swarm_comms/swarm_comms/state_broadcaster.py',
        'table': src / 'swarm_comms/swarm_comms/table.py',
        'packet': src / 'swarm_comms/swarm_comms/packet.py',
        'message': src / 'swarm_interfaces/msg/Readiness.msg',
        'centralized': src / 'swarm_bringup/launch/launch_centralized_swarm.py',
        'decentralized': src / 'swarm_bringup/launch/launch_decentralized_agent.py',
        'spacecraft': src / 'swarm_bringup/config/virtual_spacecraft.yaml',
        'benchmark': Path(__file__).resolve(),
    }
    for name, module in (('consensus', readiness_consensus_node), ('supervisor', mode_supervisor),
                         ('table', table), ('packet', packet)):
        if digest(inspect.getfile(module)) != digest(files[name]):
            raise ValueError(f'loaded {name} differs from repository source; rebuild/source this workspace')
    if not {'round_id', 'armed'} <= set(Readiness.get_fields_and_field_types()):
        raise ValueError('Readiness message is stale; rebuild and source swarm_interfaces')
    defaults = {name: declared_defaults(files[source]) for name, source in (
        ('readiness_consensus', 'consensus'), ('mode_supervisor', 'supervisor'),
        ('state_broadcaster', 'comms'))}
    spacecraft = yaml.safe_load(files['spacecraft'].read_text())
    central = launch_parameters(files['centralized'], defaults, spacecraft)
    decentralized = launch_parameters(files['decentralized'], defaults, spacecraft)
    if central != decentralized:
        raise ValueError('centralized/decentralized coordination parameters differ')
    return dict(parameters=decentralized,
                sources={name: dict(path=str(path), sha256=digest(path)) for name, path in files.items()},
                model=dict(DEFAULT_MODEL))


class OfflinePorts:
    """ROS I/O replacement only; production constructors and decisions run unchanged."""
    def declare_parameter(self, name, default):
        self.resolved[name] = self.overrides.get(name, default)
        return self.get_parameter(name)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.resolved[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(nanoseconds=EPOCH + self.sim.now + self.offset))

    def get_logger(self):
        return SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)

    def create_subscription(self, msg_type, topic, callback, qos):
        self.sim.subscriptions[self.sim.topic(self.rid, topic)].append(callback)
        return callback

    def create_publisher(self, msg_type, topic, qos):
        return SimpleNamespace(publish=lambda msg: self.sim.publish(self, topic, msg))

    def create_timer(self, period, callback):
        period_ns = ns(period)
        phase = self.sim.rng('phases', self.rid, self.role, len(self.timer_phases)).randrange(period_ns)
        self.timer_phases.append(phase)
        self.sim.repeat(phase, period_ns, callback)
        return callback

    def create_client(self, srv_type, name):
        def call(request):
            self.sim.record('service', self.rid, name=name, enabled=request.data)
            return SimpleNamespace(result=lambda: srv_type.Response(success=True))
        return SimpleNamespace(srv_name=name, wait_for_service=lambda **kw: True, call_async=call)


OfflineConsensus = type('OfflineConsensus', (OfflinePorts, readiness_consensus_node.ReadinessConsensusNode), {})
OfflineSupervisor = type('OfflineSupervisor', (OfflinePorts, mode_supervisor.ModeSupervisor), {})


def make_node(cls, sim, rid, role, overrides):
    node = cls.__new__(cls)
    node.sim, node.rid, node.role = sim, rid, role
    node.offset = sim.offsets[rid]
    node.overrides, node.resolved, node.timer_phases = dict(overrides, robot_id=rid), {}, []
    # Only construction is patched; no ROS context is initialized, including during imports.
    with patch.object(Node, '__init__', lambda *a, **kw: None):
        cls.__init__(node)
    return node


def graph(size, topology, rng):
    edges = set()
    def add(a, b):
        edges.add(tuple(sorted((a, b))))
    if topology == 'complete':
        edges = {(a, b) for a in range(size) for b in range(a + 1, size)}
    elif topology in ('chain', 'ring'):
        for a in range(size - 1):
            add(a, a + 1)
        if topology == 'ring':
            add(0, size - 1)
    elif topology == 'star':
        for b in range(1, size):
            add(0, b)
    elif topology == 'random':
        order = list(range(size))
        rng.shuffle(order)
        for i in range(1, size):
            add(order[i], rng.choice(order[:i]))
        for a in range(size):
            for b in range(a + 1, size):
                if rng.random() < 0.3:
                    add(a, b)
    else:
        raise ValueError(f'unknown topology {topology}')
    return {(a + 1, b + 1) for a, b in edges}


class Simulation:
    def __init__(self, config, size, topology, profile, seed, duration=30.0,
                 tolerance_ms=200.0, trace=False, scenario=None):
        self.identity = (size, topology, profile, seed)
        self.size, self.topology, self.profile, self.seed = self.identity
        self.config, self.model = config, config['model']
        self.horizon, self.tolerance_ns = ns(duration), ns(tolerance_ms / 1000)
        self.now, self.queue, self.serial = 0, [], 0
        self.order_rng = self.rng('ordering')
        self.subscriptions = defaultdict(list)
        self.trace = [] if trace else None
        self.triggers, self.entries = defaultdict(list), defaultdict(list)
        self.unsafe, self.snapshots = [], []
        self.max_round = defaultdict(int)
        self.network = Counter()
        self.edges = graph(size, topology, self.rng('graph'))
        setup = self.rng('inputs')
        self.available = {rid: ns(setup.uniform(0, self.model['availability_max']))
                          for rid in range(1, size + 1)}
        self.offsets = {rid: ns(self.rng('clock', rid).uniform(-self.model['clock_offset_ms'],
                           self.model['clock_offset_ms']) / 1000) if profile == 'clock' else 0
                        for rid in self.available}
        self.victim = setup.randint(1, size)
        start = setup.uniform(0, self.model['fault_start_max'])
        length = setup.uniform(self.model['partition_min'], self.model['partition_max']) \
            if profile == 'partition' else setup.uniform(self.model['perception_min'], self.model['perception_max'])
        self.fault = (ns(start), ns(start + length))
        self.loss = setup.uniform(0, self.model['loss_max']) if profile == 'loss' else 0.0
        order = list(self.available)
        setup.shuffle(order)
        self.cut = set(order[:max(1, size // 2)])
        self.scenario = scenario or {}
        self.available.update({int(k): ns(v) for k, v in self.scenario.get('available', {}).items()})
        if 'fault' in self.scenario:
            self.fault = tuple(ns(v) for v in self.scenario['fault'])
        self.consensus, self.supervisors, self.tables = {}, {}, {}
        self.inbox = defaultdict(list)
        self.link_rngs = {}
        self.last_raw_stamp = {}
        self.seq = defaultdict(int)
        params = config['parameters']
        self.comms = params['state_broadcaster']
        for rid in self.available:
            self.tables[rid] = table.NeighbourTable(rid, self.comms['stale_timeout'], self.comms['drop_timeout'])
            self.consensus[rid] = make_node(OfflineConsensus, self, rid, 'consensus', params['readiness_consensus'])
            self.supervisors[rid] = make_node(OfflineSupervisor, self, rid, 'supervisor', params['mode_supervisor'])
            for kind, hz in (('assignment', self.model['assignment_hz']), ('raw', self.model['raw_hz']),
                             ('estimated', self.model['estimated_hz']), ('state', self.comms['broadcast_rate']),
                             ('receive', self.comms['receive_rate'])):
                period = ns(1 / hz)
                phase = self.rng('phases', rid, kind).randrange(period)
                self.repeat(phase, period, lambda rid=rid, kind=kind: self.input_event(rid, kind))
        self.description = dict(identity=self.identity, edges=sorted(self.edges), available_ns=self.available,
                                clock_offsets_ns=self.offsets, fault_ns=self.fault, victim=self.victim,
                                partition_side=sorted(self.cut), packet_loss=self.loss,
                                timer_phases={rid: dict(consensus=self.consensus[rid].timer_phases,
                                                       supervisor=self.supervisors[rid].timer_phases)
                                              for rid in self.available}, scenario=self.scenario)

    def rng(self, *keys):
        return stream(*self.identity, *keys)

    @staticmethod
    def topic(rid, topic):
        return topic if topic.startswith('/') else f'/{rid}/{topic}'

    def schedule(self, when, callback):
        if when <= self.horizon:
            self.serial += 1
            heapq.heappush(self.queue, (when, self.order_rng.getrandbits(64), self.serial, callback))

    def repeat(self, when, period, callback):
        def event():
            callback()
            self.schedule(self.now + period, event)
        self.schedule(when, event)

    def record(self, event, rid=None, **data):
        if self.trace is not None:
            self.trace.append(dict(time_ns=self.now, event=event, robot=rid, **data))

    def local(self, rid, topic, msg):
        key = self.topic(rid, topic)
        callbacks = self.subscriptions.get(key, ())
        if callbacks:
            def deliver():
                for callback in callbacks:
                    callback(msg)
            self.schedule(self.now + ns(self.model['local_delay_ms'] / 1000), deliver)

    def active_fault(self):
        return self.fault[0] <= self.now < self.fault[1]

    def send(self, sender, receiver, channel, payload):
        self.network['sent'] += 1
        if self.profile == 'partition' and self.active_fault() \
                and ((sender in self.cut) != (receiver in self.cut)):
            self.network['partition_drops'] += 1
            return
        key = (sender, receiver, channel)
        if key not in self.link_rngs:
            self.link_rngs[key] = self.rng('delivery', *key)
        rng = self.link_rngs[key]
        if rng.random() < self.loss:
            self.network['loss_drops'] += 1
            return
        if self.profile == 'reference':
            delay = 0
        elif self.profile == 'latency':
            delay = ns(rng.uniform(0, self.model['stress_delay_max_ms']) / 1000)
        else:
            delay = ns(rng.uniform(self.model['delay_min_ms'], self.model['delay_max_ms']) / 1000)
        sent = self.now
        def deliver():
            self.network['delivered'] += 1
            self.record('delivery', receiver, sender=sender, channel=channel, sent_ns=sent)
            if channel == 'state':
                self.inbox[receiver].append(payload)
            else:
                self.consensus[receiver].on_peer(payload)
        self.schedule(sent + delay, deliver)

    def publish(self, node, topic, msg):
        rid = node.rid
        if topic == '/readiness':
            self.max_round[rid] = max(self.max_round[rid], msg.round_id)
            self.record('readiness', rid, ready=msg.ready, armed=msg.armed,
                        round_id=msg.round_id, action_time=msg.action_time)
            for edge in sorted(self.edges):
                if rid in edge:
                    other = edge[1] if edge[0] == rid else edge[0]
                    self.send(rid, other, 'readiness', msg)
        elif topic == 'capture_sync' and msg.data:
            self.triggers[rid].append(self.now)
            self.snapshots.append(dict(robot=rid, round_id=node.round_id, deadline=node.armed_time))
            self.check_ready('trigger', rid)
            self.record('trigger', rid, round_id=node.round_id, deadline=node.armed_time)
            self.local(rid, topic, msg)
        elif topic == 'current_mode':
            self.record('mode', rid, mode=msg.data)
            if msg.data == 'capture':
                self.entries[rid].append(self.now)
                self.check_ready('capture', rid)

    def check_ready(self, event, rid):
        unready = [other for other, agent in self.consensus.items()
                   if not agent.local_ready(agent.now())]
        if unready:
            self.unsafe.append(dict(time_ns=self.now, event=event, robot=rid, unready=unready))

    def markers(self, rid, visible=True, stamp=None):
        msg = Markers()
        msg.header.stamp = Time(nanoseconds=EPOCH + self.now + self.offsets[rid]
                               if stamp is None else stamp).to_msg()
        if visible:
            msg.ids = [rid]
            msg.markers = [CoordXY(x=self.model['distance_m'] * 100, y=0.0)]
            msg.markers_normals = [CoordXY(x=1.0, y=0.0)]
        return msg

    def input_event(self, rid, kind):
        available = self.now >= self.available[rid]
        if self.profile == 'never_ready' and rid == self.victim:
            available = False
        if kind == 'assignment':
            self.local(rid, 'assigned_ap', Int32(data=rid if available else -1))
            self.local(rid, 'assignment_locked', Bool(data=available))
        elif kind == 'raw':
            fault = rid == self.victim and self.active_fault()
            if fault and self.profile == 'perception':
                return
            stamp = self.last_raw_stamp.get(rid) if fault and self.profile == 'frozen' else None
            msg = self.markers(rid, available, stamp)
            self.last_raw_stamp[rid] = msg.header.stamp.sec * NS + msg.header.stamp.nanosec
            self.record('raw', rid, ids=list(msg.ids), stamp_ns=self.last_raw_stamp[rid])
            self.local(rid, 'targets_unf', msg)
        elif kind == 'estimated':
            self.local(rid, 'targets', self.markers(rid, available))
            self.local(rid, 'landmarks', self.markers(rid, True))
        elif kind == 'state':
            self.seq[rid] += 1
            payload = packet.decode(packet.encode(rid, self.seq[rid],
                        (EPOCH + self.now + self.offsets[rid]) / NS, 0, 0, 0, 0, 0, 0))
            for edge in sorted(self.edges):
                if rid in edge:
                    self.send(rid, edge[1] if edge[0] == rid else edge[0], 'state', payload)
        elif kind == 'receive':
            neighbour_table = self.tables[rid]
            for payload in self.inbox.pop(rid, []):
                neighbour_table.ingest(payload, t_recv=self.now / NS,
                                       now_wall=(EPOCH + self.now + self.offsets[rid]) / NS)
            neighbour_table.expire(self.now / NS)
            msg = NeighbourStates()
            for other, entry in sorted(neighbour_table.entries.items()):
                msg.neighbours.append(NeighbourState(robot_id=other,
                    age=entry.age(self.now / NS), valid=entry.valid and entry.pose_valid))
            self.local(rid, 'comms/neighbours', msg)

    def run(self):
        try:
            while self.queue:
                self.now, _, _, callback = heapq.heappop(self.queue)
                callback()
            result = evaluate(self.size, self.triggers, self.entries, self.tolerance_ns)
            result.update(unsafe=bool(self.unsafe), unsafe_events=self.unsafe,
                          duplicate_triggers=any(len(v) > 1 for v in self.triggers.values()),
                          differing_rounds=len({v['round_id'] for v in self.snapshots}) > 1,
                          differing_deadlines=len({v['deadline'] for v in self.snapshots}) > 1,
                          max_round=max(self.max_round.values(), default=0),
                          restrained=self.profile == 'never_ready' and not self.triggers and not self.entries)
        except Exception as exc:
            result = dict(outcome='execution_error', error=f'{type(exc).__name__}: {exc}')
        result.update(size=self.size, topology=self.topology, profile=self.profile, seed=self.seed,
                      network=dict(self.network))
        return result


def evaluate(size, triggers, entries, tolerance_ns):
    def spread(events):
        times = [values[0] for values in events.values() if values]
        return max(times) - min(times) if len(times) > 1 else 0 if times else None
    entered = sum(bool(v) for v in entries.values())
    mode_spread = spread(entries)
    if entered == size:
        outcome = 'synchronized_capture' if mode_spread <= tolerance_ns else 'unsynchronized_capture'
    else:
        outcome = 'partial_capture' if entered else 'no_capture'
    return dict(outcome=outcome, entered=entered, fired=sum(bool(v) for v in triggers.values()),
                firing_fraction=sum(bool(v) for v in triggers.values()) / size,
                trigger_spread_ns=spread(triggers), mode_spread_ns=mode_spread,
                first_capture_ns=min((v[0] for v in entries.values() if v), default=None),
                full_capture_ns=max((v[0] for v in entries.values() if v), default=None)
                if entered == size else None,
                trigger_times_ns=dict(triggers), capture_times_ns=dict(entries))


def wilson(successes, count):
    if not count:
        return None
    z = 1.959963984540054
    p = successes / count
    denominator = 1 + z * z / count
    centre = (p + z * z / (2 * count)) / denominator
    radius = z * math.sqrt(p * (1 - p) / count + z * z / (4 * count * count)) / denominator
    return [max(0.0, centre - radius), min(1.0, centre + radius)]


def aggregate(results):
    groups = defaultdict(list)
    for result in results:
        category = 'normal' if result['profile'] in NORMAL else 'stress'
        groups[(category, 'all', 'all', 'all')].append(result)
        groups[(category, result['size'], 'all', 'all')].append(result)
        groups[(category, result['size'], result['topology'], result['profile'])].append(result)
    summary = []
    for key, rows in groups.items():
        counts = Counter(r['outcome'] for r in rows)
        success = counts['synchronized_capture']
        summary.append(dict(category=key[0], size=key[1], topology=key[2], profile=key[3],
                            trials=len(rows), counts=dict(counts), success_rate=success / len(rows),
                            success_ci95=wilson(success, len(rows)),
                            unsafe_trials=sum(bool(r.get('unsafe')) for r in rows),
                            restrained_trials=sum(bool(r.get('restrained')) for r in rows)))
    return summary


def find_root():
    for origin in (Path(__file__).resolve(), Path.cwd()):
        for candidate in (origin, *origin.parents):
            if (candidate / 'ros2_swarm/src/swarm_bringup').is_dir():
                return candidate
    raise ValueError('cannot locate repository; pass --repo-root')


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo-root', type=Path)
    p.add_argument('--num-seeds', type=int, default=150, help='seeds per size/topology/profile cell')
    p.add_argument('--seed-start', type=int, default=0)
    p.add_argument('--sizes', type=int, nargs='+', default=list(range(2, 9)))
    p.add_argument('--topologies', nargs='+', choices=TOPOLOGIES, default=list(TOPOLOGIES))
    p.add_argument('--profiles', nargs='+', choices=PROFILES, default=list(PROFILES))
    p.add_argument('--duration', type=float, default=30.0)
    p.add_argument('--sync-tolerance-ms', type=float, default=200.0)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--replay', type=Path, help='replay a saved failure trace or case descriptor')
    p.add_argument('--trace', action='store_true', help='save timelines for successful cases too')
    p.add_argument('--progress-every', type=int, default=25)
    for name in DEFAULT_MODEL:
        p.add_argument('--' + name.replace('_', '-'), type=float, default=None,
                       help=f'synthetic model setting; default {DEFAULT_MODEL[name]}')
    return p


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')


def markdown(summary, elapsed, tolerance_ms=200.0):
    lines = ['# Software coordination simulation', '',
             f'Success: all configured robots enter CAPTURE within {tolerance_ms:g} ms.',
             'No DDS, hardware scheduling, auction, or physical dynamics are simulated.',
             f'Runtime: {elapsed:.2f} s.', '',
             '| Category | Size | Topology | Profile | Trials | Sync success | 95% CI | Unsafe | Restrained | Errors |',
             '|---|---|---|---|---:|---:|---|---:|---:|---:|']
    for row in summary:
        lo, hi = row['success_ci95']
        lines.append(f"| {row['category']} | {row['size']} | {row['topology']} | {row['profile']} | "
                     f"{row['trials']} | {row['success_rate']:.1%} | {lo:.1%}–{hi:.1%} | "
                     f"{row['unsafe_trials']} | {row['restrained_trials']} | "
                     f"{row['counts'].get('execution_error', 0)} |")
    lines += ['', 'No capture is not a synchronization success. Never-ready restraint is reported separately.',
              'Unsafe means a trigger or CAPTURE transition occurred while a required agent was locally unready.',
              'Stress and normal-operation rates are separate; intervals describe this sample distribution only.']
    return '\n'.join(lines) + '\n'


def main(argv=None):
    args = parser().parse_args(argv)
    started = time.monotonic()
    try:
        if args.num_seeds < 1 or args.seed_start < 0 or any(n not in range(2, 9) for n in args.sizes):
            raise ValueError('use positive seed counts, nonnegative seed starts, and sizes 2–8')
        if not math.isfinite(args.duration) or args.duration <= 0 or not math.isfinite(args.sync_tolerance_ms) \
                or args.sync_tolerance_ms < 0 or args.progress_every < 1:
            raise ValueError('duration must be positive; tolerance nonnegative; progress interval positive')
        config = load_config(args.repo_root or find_root())
        for name in DEFAULT_MODEL:
            value = getattr(args, name)
            if value is not None:
                config['model'][name] = value
        if any(not math.isfinite(v) or v < 0 for v in config['model'].values()):
            raise ValueError('model settings must be finite and nonnegative')
        if any(config['model'][name] <= 0 for name in ('raw_hz', 'estimated_hz', 'assignment_hz')):
            raise ValueError('input rates must be positive')
        if any(ns(1 / config['model'][name]) < 1 for name in ('raw_hz', 'estimated_hz', 'assignment_hz')):
            raise ValueError('input periods must be at least one nanosecond')
        for prefix in ('delay', 'perception', 'partition'):
            low, high = ('delay_min_ms', 'delay_max_ms') if prefix == 'delay' else (prefix + '_min', prefix + '_max')
            if config['model'][low] > config['model'][high]:
                raise ValueError(f'{low} must not exceed {high}')
        if config['model']['loss_max'] > 1:
            raise ValueError('loss-max must not exceed 1')
        replay = json.loads(args.replay.read_text()) if args.replay else None
        if replay:
            saved = replay['config']
            if {k: v['sha256'] for k, v in saved['sources'].items()} != \
                    {k: v['sha256'] for k, v in config['sources'].items()}:
                raise ValueError('replay source hashes differ from current sources')
            if saved['parameters'] != config['parameters']:
                raise ValueError('replay production parameters differ')
            config = saved
            cases = [tuple(replay['case']['identity'])]
            args.duration, args.sync_tolerance_ms = replay['duration'], replay['tolerance_ms']
        else:
            cases = [(size, topology, profile, seed)
                     for size in sorted(set(args.sizes)) for topology in sorted(set(args.topologies))
                     for profile in sorted(set(args.profiles))
                     for seed in range(args.seed_start, args.seed_start + args.num_seeds)]
        if replay:
            if len(cases[0]) != 4 or cases[0][0] not in range(2, 9) \
                    or cases[0][1] not in TOPOLOGIES or cases[0][2] not in PROFILES:
                raise ValueError('unsupported replay case identity')
        args.output_dir.mkdir(parents=True, exist_ok=False)
        traces = args.output_dir / 'traces'
        traces.mkdir()
        manifest = dict(label='software coordination simulation', config=config,
                        sizes=args.sizes, topologies=args.topologies, profiles=args.profiles,
                        num_seeds=args.num_seeds, seed_start=args.seed_start,
                        duration=args.duration, tolerance_ms=args.sync_tolerance_ms,
                        trials=len(cases), python=sys.version)
        write_json(args.output_dir / 'manifest.json', manifest)
        results = []
        print(f'Software coordination simulation: {len(cases)} trials', flush=True)
        with (args.output_dir / 'trials.csv').open('w', newline='') as handle:
            writer = None
            for index, (size, topology, profile, seed) in enumerate(cases, 1):
                sim = None
                try:
                    sim = Simulation(config, size, topology, profile, seed, args.duration,
                                     args.sync_tolerance_ms, trace=bool(replay) or args.trace,
                                     scenario=replay['case'].get('scenario') if replay else None)
                    result = sim.run()
                except Exception as exc:
                    result = dict(size=size, topology=topology, profile=profile, seed=seed,
                                  outcome='execution_error', error=f'{type(exc).__name__}: {exc}')
                results.append(result)
                if result['outcome'] != 'synchronized_capture' or result.get('unsafe') or args.trace or replay:
                    if sim is not None:
                        if sim.trace is None:
                            replay_sim = Simulation(config, size, topology, profile, seed,
                                                    args.duration, args.sync_tolerance_ms, trace=True)
                            repeated = replay_sim.run()
                            if repeated != result:
                                raise RuntimeError(f'non-deterministic replay for {sim.identity}')
                            sim = replay_sim
                        write_json(traces / f'{size}-{topology}-{profile}-{seed}.json',
                                   dict(config=config, duration=args.duration,
                                        tolerance_ms=args.sync_tolerance_ms, case=sim.description,
                                        result=result, events=sim.trace))
                    else:
                        write_json(traces / f'{size}-{topology}-{profile}-{seed}.json',
                                   dict(config=config, duration=args.duration,
                                        tolerance_ms=args.sync_tolerance_ms,
                                        case=dict(identity=(size, topology, profile, seed)),
                                        result=result, events=[]))
                row = {key: result.get(key) for key in (
                    'size', 'topology', 'profile', 'seed', 'outcome', 'entered', 'fired',
                    'firing_fraction', 'trigger_spread_ns', 'mode_spread_ns', 'first_capture_ns',
                    'full_capture_ns', 'max_round', 'unsafe', 'duplicate_triggers',
                    'differing_rounds', 'differing_deadlines', 'restrained', 'error')}
                if writer is None:
                    writer = csv.DictWriter(handle, fieldnames=list(row))
                    writer.writeheader()
                writer.writerow(row)
                if index % args.progress_every == 0 or index == len(cases):
                    handle.flush()
                    print(f'{index}/{len(cases)} trials; {time.monotonic() - started:.1f} s', flush=True)
        elapsed = time.monotonic() - started
        summary = aggregate(results)
        write_json(args.output_dir / 'summary.json', dict(label=manifest['label'], runtime_s=elapsed,
                   tolerance_ms=args.sync_tolerance_ms, groups=summary))
        (args.output_dir / 'summary.md').write_text(markdown(summary, elapsed, args.sync_tolerance_ms))
        print(f'Results: {args.output_dir.resolve()}', flush=True)
        return 1 if any(r['outcome'] == 'execution_error' for r in results) else 0
    except (ValueError, OSError, RuntimeError, KeyError, TypeError, SyntaxError) as exc:
        print(f'consensus_benchmark: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
