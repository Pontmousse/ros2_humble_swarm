"""Offline timing, fidelity, replay, and scoring checks for the batch runner."""
from copy import deepcopy
import heapq
import json
from pathlib import Path
from unittest.mock import patch

import pytest

from swarm_controller import consensus_benchmark as bench


@pytest.fixture(scope='module')
def config():
    return bench.load_config(Path(__file__).resolve().parents[4])


def until(sim, seconds):
    while sim.queue and sim.queue[0][0] <= bench.ns(seconds):
        sim.now, _, _, callback = heapq.heappop(sim.queue)
        callback()


@pytest.mark.parametrize('spread,outcome', [(199_999_999, 'synchronized_capture'),
                                          (200_000_000, 'synchronized_capture'),
                                          (200_000_001, 'unsynchronized_capture')])
def test_sync_boundary(spread, outcome):
    result = bench.evaluate(2, {1: [0], 2: [spread]}, {1: [0], 2: [spread]}, 200_000_000)
    assert result['outcome'] == outcome


def test_missing_agent_never_shrinks_required_roster():
    result = bench.evaluate(3, {1: [0], 2: [1]}, {1: [0], 2: [1]}, bench.ns(.2))
    assert result['outcome'] == 'partial_capture'
    assert result['firing_fraction'] == 2 / 3
    assert result['full_capture_ns'] is None
    assert bench.evaluate(3, {}, {}, bench.ns(.2))['outcome'] == 'no_capture'


@pytest.mark.parametrize('topology', bench.TOPOLOGIES)
def test_graphs_are_connected(topology):
    edges = bench.graph(8, topology, bench.stream(topology))
    reached = {1}
    while True:
        expanded = reached | {a for edge in edges if reached.intersection(edge) for a in edge}
        if expanded == reached:
            break
        reached = expanded
    assert reached == set(range(1, 9))


def test_production_initialization_and_no_network(config):
    with patch('rclpy.init', side_effect=AssertionError('must not initialize DDS')), \
            patch('socket.socket', side_effect=AssertionError('must not open sockets')):
        sim = bench.Simulation(config, 3, 'chain', 'reference', 0, duration=1)
        assert sim.run()['outcome'] != 'execution_error'
    assert sim.consensus[1].min_lead == .5
    assert sim.supervisors[1].require_capture_sync
    assert sim.supervisors[1].targets.detect_frames == 3
    assert len({a.timer_phases[0] for a in sim.consensus.values()}) == 3
    assert sim.consensus[1].timer_phases != sim.supervisors[1].timer_phases
    assert 'local_ready' not in bench.OfflinePorts.__dict__


def test_replay_and_case_order_are_exact(config):
    def run(seed):
        sim = bench.Simulation(config, 3, 'chain', 'latency', seed, duration=3, trace=True)
        return sim.run(), sim.trace, sim.description
    first = run(150)
    run(1000)
    assert first == run(150)


@pytest.mark.parametrize('last_ready', [.4, 2.2])
def test_three_robot_normal_and_late_readiness(config, last_ready):
    sim = bench.Simulation(config, 3, 'chain', 'reference', 0, duration=8,
                           scenario={'available': {1: 0., 2: .2, 3: last_ready}}, trace=True)
    result = sim.run()
    assert result['entered'] == 3
    assert not result['unsafe']
    if last_ready == 2.2:
        assert result['max_round'] > 0
        assert result['first_capture_ns'] >= bench.ns(last_ready)


def test_neighbour_entries_expire_naturally(config):
    sim = bench.Simulation(config, 2, 'complete', 'partition', 3, duration=6,
                           scenario={'fault': [2., 5.]})
    sim.cut = {1}
    until(sim, 1.9)
    assert sim.tables[1].entries[2].valid
    until(sim, 2.2)
    assert sim.tables[1].entries[2].valid  # Last received packet is still fresh.
    until(sim, 2.8)
    assert not sim.tables[1].entries[2].valid
    assert not sim.consensus[1].live_ids
    until(sim, 5.3)
    assert sim.tables[1].entries[2].valid


@pytest.mark.parametrize('profile', ['frozen', 'perception'])
def test_raw_liveness_fails_while_estimated_targets_continue(config, profile):
    sim = bench.Simulation(config, 2, 'complete', profile, 4, duration=4,
                           scenario={'available': {1: 0., 2: 0.}, 'fault': [1., 3.]}, trace=True)
    until(sim, 2.)
    victim = sim.consensus[sim.victim]
    assert victim.ap_in_range()
    assert not victim.perception_live(victim.now())
    assert not victim.local_ready(victim.now())
    until(sim, 3.5)
    assert victim.local_ready(victim.now())
    if profile == 'frozen':
        stamps = [e['stamp_ns'] for e in sim.trace if e['event'] == 'raw'
                  and e['robot'] == sim.victim and bench.ns(1.2) < e['time_ns'] < bench.ns(2.8)]
        assert len(stamps) > 2 and len(set(stamps)) == 1


def test_variable_delay_can_reorder_packets(config):
    sim = bench.Simulation(config, 3, 'complete', 'latency', 10, duration=3, trace=True)
    sim.run()
    sends = [e['sent_ns'] for e in sim.trace if e['event'] == 'delivery'
             and e['sender'] == 1 and e['robot'] == 2 and e['channel'] == 'state']
    assert any(a > b for a, b in zip(sends, sends[1:]))


def test_aggregation_preserves_errors_and_separates_safety():
    rows = [dict(size=2, topology='chain', profile='nominal', outcome='synchronized_capture', unsafe=True),
            dict(size=2, topology='chain', profile='nominal', outcome='execution_error'),
            dict(size=2, topology='chain', profile='never_ready', outcome='no_capture', restrained=True)]
    summaries = bench.aggregate(rows)
    normal = next(r for r in summaries if r['category'] == 'normal' and r['size'] == 'all')
    stress = next(r for r in summaries if r['category'] == 'stress' and r['size'] == 'all')
    assert normal['trials'] == 2 and normal['success_rate'] == .5
    assert normal['unsafe_trials'] == 1 and normal['counts']['execution_error'] == 1
    assert stress['success_rate'] == 0 and stress['restrained_trials'] == 1
    assert bench.wilson(0, 0) is None
    assert bench.wilson(5, 10) == pytest.approx([.236593090512564, .763406909487436])


def test_parameter_mismatch_is_rejected(config, tmp_path):
    source = Path(config['sources']['decentralized']['path']).read_text()
    target = tmp_path / 'launch.py'
    target.write_text(source.replace('"action_lead": 2.0', '"action_lead": 3.0'))
    defaults = {name: bench.declared_defaults(config['sources'][key]['path']) for name, key in (
        ('readiness_consensus', 'consensus'), ('mode_supervisor', 'supervisor'), ('state_broadcaster', 'comms'))}
    parsed = bench.launch_parameters(target, defaults, {})
    assert parsed != config['parameters']
    with patch.object(bench, 'launch_parameters', side_effect=[config['parameters'], parsed]):
        with pytest.raises(ValueError, match='parameters differ'):
            bench.load_config(Path(__file__).resolve().parents[4])


def test_source_mismatch_is_rejected(config, tmp_path):
    different = tmp_path / 'stale.py'
    different.write_text('stale installed module')
    with patch.object(bench.inspect, 'getfile', return_value=str(different)):
        with pytest.raises(ValueError, match='differs from repository'):
            bench.load_config(Path(__file__).resolve().parents[4])


def test_saved_replay_checks_hashes(config, tmp_path):
    bad = deepcopy(config)
    bad['sources']['consensus']['sha256'] = 'wrong'
    replay = tmp_path / 'bad.json'
    replay.write_text(json.dumps(dict(config=bad)))
    assert bench.main(['--replay', str(replay), '--output-dir', str(tmp_path / 'out')]) == 2
    assert not (tmp_path / 'out').exists()


def test_cli_outputs_and_single_case_replay(config, tmp_path):
    output, replay_output = tmp_path / 'batch', tmp_path / 'replay'
    args = ['--num-seeds', '1', '--seed-start', '150', '--sizes', '2', '--profiles', 'nominal',
            '--topologies', 'chain', '--duration', '2', '--trace', '--output-dir', str(output)]
    assert bench.main(args) == 0
    trace = next((output / 'traces').glob('*.json'))
    assert bench.main(['--replay', str(trace), '--output-dir', str(replay_output)]) == 0
    original = json.loads(trace.read_text())
    repeated = json.loads(next((replay_output / 'traces').glob('*.json')).read_text())
    assert original['result'] == repeated['result']
    assert original['events'] == repeated['events']
    assert (output / 'trials.csv').exists()
    assert (output / 'summary.json').exists()
    assert 'Software coordination simulation' in (output / 'summary.md').read_text()
