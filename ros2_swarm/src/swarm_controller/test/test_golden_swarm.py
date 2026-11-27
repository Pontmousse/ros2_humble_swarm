"""The Phase 5 classes must reproduce the old swarm-logic nodes exactly.

golden_swarm.json was recorded from ApAuctionNode, ReadinessConsensusNode, ModeSupervisor and GeometricContact at
commit b0d3085 (pre-Phase-5) through their real __init__/callbacks/timers, driven by golden_swarm.py: the same
driver runs here against Auction, ReadinessConsensus, ModeSupervisor and GeometricContact, and every published
message and service call must match, including the consensus one-shot fire and the supervisor's resend of lost
replies. Peers in the consensus scenario echo what the robot publishes, so any divergence also changes the inputs.
"""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import golden_swarm as G
from swarm_controller.ap_auction import Auction
from swarm_controller.geometric_contact import GeometricContact
from swarm_controller.mode_supervisor import ModeSupervisor
from swarm_controller.readiness_consensus import ReadinessConsensus

FIXTURE = json.loads((Path(__file__).parent / 'golden_swarm.json').read_text())
EVENTS = dict(auction=G.auction_events, consensus=G.consensus_events, supervisor=G.supervisor_events,
              geometric=G.geometric_events)


def build(name, world, params):
    p = lambda n, d: params.get(n, d)  # noqa: E731
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=world.now),
                           get_logger=lambda: SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None,
                                                              warning=lambda *a, **k: None, error=lambda *a, **k: None),
                           create_timer=world.create_timer, destroy_timer=world.destroy_timer,
                           create_client=lambda t, n: world.client(n))
    if name == 'auction':
        return Auction(node, p, world.pub('/ap_bids'), world.pub('assigned_ap'), world.pub('assignment_locked'))
    if name == 'consensus':
        return ReadinessConsensus(node, p, world.pub('/readiness'), world.pub('capture_sync'))
    if name == 'supervisor':
        return ModeSupervisor(node, p, world.pub('current_mode'))
    return GeometricContact(p, world.pub('contact_detected'))


@pytest.mark.parametrize('name', sorted(FIXTURE))
def test_matches_the_old_node(name):
    want = FIXTURE[name]
    world = G.World()
    events = EVENTS[name](world)
    obj = build(name, world, want['params'])
    assert world.log == want['init']
    world.log = []
    got = G.run(world, obj, events)
    assert len(got) == len(want['outputs'])
    for k, (g, w) in enumerate(zip(got, want['outputs'])):
        assert json.loads(json.dumps(g)) == w, (name, k, events[k][0] if k < len(events) else None)
    assert sum(len(o) for o in want['outputs']) > 15
