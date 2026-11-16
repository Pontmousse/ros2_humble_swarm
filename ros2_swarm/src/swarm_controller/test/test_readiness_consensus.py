"""Exercise capture coordination with simulated clocks, without joining DDS."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from rclpy.time import Time

from swarm_controller.readiness_consensus_node import ReadinessConsensusNode
from swarm_interfaces.msg import Readiness


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(deepcopy(msg))


class Agent:
    round = ReadinessConsensusNode.round
    reset_round = ReadinessConsensusNode.reset_round
    on_peer = ReadinessConsensusNode.on_peer
    publish_readiness = ReadinessConsensusNode.publish_readiness
    stamp_sec = ReadinessConsensusNode.stamp_sec

    def __init__(self, robot_id=1, neighbours=(2,)):
        self.robot_id = robot_id
        self.live_ids = set(neighbours)
        self.participants = frozenset(neighbours)
        self.assigned_ap = self.proposal_ap = robot_id
        self.peers = {}
        self.round_id = 0
        self.my_t = self.armed_time = self.hold = 0.0
        self.armed = self.fired = False
        self.ready = True
        self.time = 100.0
        self.action_lead, self.min_lead, self.time_tol = 2.0, 0.5, 0.05
        self.neighbor_timeout, self.min_consensus_hold, self.dt = 0.5, 0.3, 0.2
        self.ready_pub, self.sync_pub, self.debug_pub = Publisher(), Publisher(), Publisher()

    def now(self):
        return self.time

    def local_ready(self, now):
        return self.ready

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(nanoseconds=round(self.time * 1e9)))

    def get_logger(self):
        return SimpleNamespace(info=lambda message: None)


def peer(agent, action_time, *, robot_id=2, ready=True, armed=False, round_id=None):
    msg = Readiness()
    msg.header.stamp = agent.get_clock().now().to_msg()
    msg.robot_id, msg.ready, msg.armed = robot_id, ready, armed
    msg.action_time = float(action_time)
    msg.round_id = agent.round_id if round_id is None else round_id
    agent.on_peer(msg)


def arm(agent):
    for _ in range(2):
        peer(agent, 102.0, armed=True)
        agent.round()
        agent.time += 0.2
    assert agent.armed
    assert agent.armed_time == 102.0


def test_minimum_consensus_only_reduces_within_a_round():
    agent = Agent()
    peer(agent, 101.5)
    agent.round()
    assert agent.my_t == 101.5
    agent.time += 0.2
    peer(agent, 103.0)
    agent.round()
    assert agent.my_t == 101.5


def test_frozen_deadline_fires_once_without_min_lead_pushback():
    agent = Agent()
    arm(agent)
    for step in range(2, 14):
        agent.time = 100.0 + step * 0.2
        peer(agent, 102.0, armed=True)
        agent.round()
        assert agent.my_t == agent.armed_time == 102.0
        assert len(agent.sync_pub.messages) == int(agent.time >= 102.0)
    assert agent.fired


@pytest.mark.parametrize('failure', ['local', 'peer', 'missing', 'isolated', 'assignment'])
def test_readiness_or_membership_loss_cancels_before_firing(failure):
    agent = Agent()
    arm(agent)
    agent.time = 101.8
    if failure == 'local':
        agent.ready = False
    elif failure == 'isolated':
        agent.live_ids.clear()
    elif failure == 'assignment':
        agent.assigned_ap = 7
    if failure != 'missing':
        peer(agent, 102.0, ready=failure != 'peer', armed=True)
    agent.round()
    assert not agent.armed
    assert not agent.sync_pub.messages
    assert agent.round_id == 1


def test_all_valid_neighbours_must_supply_readiness():
    agent = Agent(neighbours=(2, 3))
    for step in range(4):
        agent.time = 100.0 + step * 0.2
        peer(agent, 102.0, armed=True)
        agent.round()
    assert not agent.armed


def test_expired_negotiation_restarts_and_ignores_previous_round():
    agent = Agent()
    peer(agent, 100.4)
    agent.round()
    assert agent.round_id == 1
    assert agent.my_t == 0.0
    agent.time += 0.2
    peer(agent, 100.4, round_id=0)
    agent.round()
    assert agent.my_t == pytest.approx(102.2)
    assert not agent.armed


def test_later_round_withdraws_frozen_deadline():
    agent = Agent()
    arm(agent)
    peer(agent, 104.0, round_id=3)
    agent.round()
    assert agent.round_id == 3
    assert not agent.armed
    assert agent.armed_time == 0.0


def test_uncommitted_peer_prevents_execution():
    agent = Agent()
    arm(agent)
    agent.time = 102.0
    peer(agent, 102.0, armed=False)
    agent.round()
    assert not agent.sync_pub.messages
    assert agent.round_id == 1


@pytest.mark.parametrize('topology', [{1: (2, 3), 2: (1, 3), 3: (1, 2)},
                                     {1: (2,), 2: (1, 3), 3: (2,)}])
def test_delayed_multi_robot_negotiation_restarts_then_captures(topology):
    agents = [Agent(rid, neighbours) for rid, neighbours in topology.items()]
    # Deliver each broadcast on the next tick, including through the line graph.
    previous = []
    for step in range(35):
        now = 100.0 + step * 0.2
        for agent in agents:
            agent.time = now
            agent.ready = now >= {1: 100.0, 2: 100.4, 3: 102.2}[agent.robot_id]
            for msg in previous:
                agent.on_peer(msg)
        for agent in agents:
            agent.round()
        previous = [agent.ready_pub.messages[-1] for agent in agents]
    assert all(agent.fired for agent in agents)
    assert all(len(agent.sync_pub.messages) == 1 for agent in agents)
    assert len({agent.armed_time for agent in agents}) == 1
    assert all(agent.round_id > 0 for agent in agents)
