# Capture synchronization findings and deferred improvements

Recorded: 2026-11-13 (America/Toronto).

Follow-up, 2026-11-15: [capture preparation](virtual_spacecraft_hil.md#capture-preparation)
and a 2-second perception-freshness timeout have been added. The measurements below
retain their original 500 ms timeout and source identifiers; the distributed
commitment/recovery improvements remain deferred.

## Decision and research objective

Keep the current protocol for hardware evaluation. Treat **1 second** between the
first and last required robot entering CAPTURE as the working synchronization
criterion. Defer the protocol improvements below until hardware observations
justify revisiting them. This document records findings; it does not change node
behavior, launch parameters, or the benchmark's default 200 ms threshold.

The research objective remains **leaderless decentralized coordination over a
neighbour-to-neighbour communication graph**. A shared access point or mothership
relay can provide transport without making the decision process centralized.
Every agent still makes its own decision. Direct all-to-all radio coverage is not
required in principle, but the current protocol's local checks do not establish
whole-team commitment on arbitrary sparse graphs.

A fixed team roster, forwarded participant status, distributed commitment checks,
and cancellation/recovery messages can all preserve decentralization. They add
communication and negotiation time; requiring the whole team deliberately means
an unavailable member can block the attempt.

## Experiment boundary and reproduction

These are **software coordination simulation** results, not hardware timing or
spaceflight validation. The runner executes production readiness-consensus and
mode-supervisor initialization and callbacks through offline ROS I/O adapters.
It exercises real perception-liveness, freshness, arming, firing, and mode-decision
logic with synthetic inputs. It does not simulate auction outcomes, physical
motion, cameras, DDS scheduling, or controller/motor response.

See [the benchmark guide](consensus_benchmark.md) for the model, CLI, profiles, and
limits. Production timing used in these runs: consensus and supervisor at 5 Hz,
initial action lead 2 s, minimum lead 500 ms, proposal disagreement tolerance
50 ms, consensus hold 300 ms, perception freshness 500 ms, readiness-message
freshness 500 ms, and neighbour-state staleness 500 ms. Timer phases were
independent. Nominal link delay was uniform 53.6–103.6 ms, an approximation centered
on the repository's measured state-transport lag, not a measured DDS distribution.

Two batches ran for 30 simulated seconds per trial, using seeds starting at 0:

| Batch | Coverage | Trials |
|---|---|---:|
| Nominal | Complete graph; sizes 2–8; 150 seeds per size | 1,050 |
| Matrix smoke | Sizes 2–8; five topologies; nine profiles; one seed per cell | 315 |

The 35 regression checks passed, the package and installed entry point were
verified, and neither batch reported simulator execution errors. A saved unsafe
seven-robot trace replayed with identical results and event timeline. These checks
validate the runner's behavior; they do not prove the protocol correct.

Reproduce with the matching source and built messages:

```bash
ros2 run swarm_controller consensus_benchmark \
  --num-seeds 150 --topologies complete --profiles nominal \
  --output-dir /tmp/consensus-nominal-150

ros2 run swarm_controller consensus_benchmark \
  --num-seeds 1 --output-dir /tmp/consensus-matrix-smoke
```

Use `--sync-tolerance-ms 1000` to score against 1 second. This changes only the
benchmark criterion, not consensus `time_tol`, action lead, or production timers.
The comparisons below rescore saved timestamps; no algorithm parameters changed.

## Nominal results and timing distribution

All 1,050 nominal complete-graph trials achieved full-team CAPTURE entry, with
no readiness-safety flags. All `capture_sync` trigger spreads were within 200 ms;
the maximum was **199.975 ms**. CAPTURE-mode entry includes the additional
independently phased supervisor timer, so its spread was larger.

| CAPTURE-mode entry spread | Trials |
|---|---:|
| ≤200 ms | 568 |
| >200–250 ms | 242 |
| >250–300 ms | 167 |
| >300–350 ms | 66 |
| >350–400 ms | 7 |
| >400 ms | 0 |

At 200 ms, 568/1,050 trials passed (**54.1%**). At 1 second, all 1,050 passed.
The overall CAPTURE spread had median **192.694 ms**, 95th percentile **308.583 ms**,
and maximum **369.455 ms**. Among the 482 cases exceeding 200 ms, the median was
**249.770 ms** and the 95th percentile was **323.795 ms**.

| Swarm size | Pass at ≤200 ms | Pass at ≤1 s |
|---|---:|---:|
| 2 | 123/150 | 150/150 |
| 3 | 112/150 | 150/150 |
| 4 | 95/150 | 150/150 |
| 5 | 74/150 | 150/150 |
| 6 | 69/150 | 150/150 |
| 7 | 58/150 | 150/150 |
| 8 | 37/150 | 150/150 |

The declining 200 ms pass rate concerns timing spread in this batch, not missing
participants. A 1-second target removes those nominal timing failures. Whether
that tolerance suits a physical capture maneuver remains an application assumption
to evaluate against actual target motion and contact behavior.

## Sparse-graph and fault results

Each row below contains 35 smoke cases: seven swarm sizes × five graph families,
with seed 0. These counts diagnose mechanisms, not statistically established fault
probabilities. All full-team captures in this matrix had mode spread below 1 s;
the maximum was **416.703 ms**.

| Profile | Full-team capture within 1 s | Partial capture | No capture |
|---|---:|---:|---:|
| Zero-delay reference | 35 | 0 | 0 |
| Nominal latency | 33 | 2 | 0 |
| Clock offsets | 35 | 0 | 0 |
| Temporary raw-perception loss | 34 | 1 | 0 |
| Frozen raw timestamps | 33 | 2 | 0 |
| Packet loss | 27 | 8 | 0 |
| Temporary partition | 30 | 5 | 0 |
| Variable delay 0–600 ms | 1 | 7 | 27 |
| One agent never ready | 0 | 2 | 33 |

Of the 60 no-capture cases, 33 were correct restraint with a never-ready member;
27 high-delay cases did not complete within the 30-second horizon. The 27 partial
captures remain failures regardless of a wider timing tolerance. Partial groups
can act seconds apart: the sub-417 ms figure applies only to full-team captures.

Four cases were flagged unsafe: two frozen-perception cases and two never-ready
cases. Here, "unsafe" means a trigger or CAPTURE entry occurred while a required
agent was locally unready. This narrow software flag is not a complete physical
safety assessment; partial capture can also be problematic when everyone is ready.

## Failure mechanisms observed in traces

### A distant unready member does not reliably block execution

Case `7-chain-never_ready-0`: graph `1 ↔ 2 ↔ 3 ↔ 4 ↔ 5 ↔ 6 ↔ 7`; robot 6 never
became ready. Robot 2 triggered at **6.148 s**, and robot 1 at **6.235 s**, both in
round 1. Other robots kept negotiating. Only two entered CAPTURE.

A robot advertises its own local readiness. Neighbour readiness and armed flags
do not carry a complete, fresh record of all required participants. The minimum
timestamp can propagate across the graph without whole-team readiness doing so.
Membership is also derived from currently valid neighbours, rather than a fixed
capture roster, so disconnection can let a subgroup proceed.

### Different portions of the team can execute different rounds

Case `6-chain-nominal-0` had nominal delay and no injected fault. Robot 1 triggered
at **2.635 s** in round 0. Robots 5 and 6 triggered at **11.369 s** and **11.476 s**
in round 5. Robots 2, 3, and 4 did not trigger during the trial. CAPTURE entries
among the acting robots spanned approximately **9.023 s**.

New rounds and cancellation propagate with delay. Local commitment can therefore
survive long enough to execute while another part of the graph has moved on.
Increasing the allowed synchronization spread does not repair this split.

### Readiness withdrawal can arrive too late

Case `4-star-frozen-0` used robot 1 as the centre. Its camera froze at **5.685 s**;
the last advancing raw timestamp was **5.613 s**. At **5.988 s**, it still advertised
ready and armed. The three leaves triggered at **6.145–6.201 s**. The centre's next
consensus tick, at **6.188 s**, detected lost readiness and withdrew in round 3.

The perception timeout, periodic checking, and network delay leave a window in
which peers act on older ready/armed messages. Some leaves had already fired;
others had not yet received the withdrawal. Cancelling locally cannot undo an
already executed remote action.

Packet loss can cause a similar split without perception failure. In
`4-ring-loss-0`, with roughly 15.1% packet loss, robots 2–4 triggered at
**7.569–7.653 s**, while robot 1 cancelled at **7.715 s**. A last-moment missing
peer condition can stop one robot after others have executed.

### High delay prevents stable negotiation

The high-delay profile permits 600 ms delivery while readiness and neighbour
freshness limits are 500 ms. Messages can arrive stale; neighbour validity changes,
agreement breaks, and attempts restart. Case `2-ring-latency-0` reached **round 20**
without triggering within 30 seconds, despite repeated ready/armed advertisements.
A longer initial action lead alone does not fix stale-message rejection.

### Partial execution lacks coordinated recovery

After firing, an agent keeps advertising its old round and frozen deadline, while
non-firing agents can continue proposing newer rounds. The six-robot nominal trace
ended with robots advertising rounds 0, 5, and 17. This prevents treating ordinary
retry as a coherent recovery procedure after part of the team has already acted.

## Suggested improvements — deferred

These are candidates for later work, not implemented changes or a finalized protocol.
They can all remain leaderless and decentralized.

1. **Fixed capture-team roster.** Keep required participation separate from the
   communication-neighbour list. A missing required member must not silently
   disappear from the capture decision.
2. **Propagate participant-specific status.** Forward origin identity, assignment,
   readiness, attempt/version, and freshness across neighbours. Forwarding an old
   report must not refresh its origin's liveness.
3. **Whole-team commitment.** Separate ready, prepared, committed, and executed
   states. Every robot independently verifies evidence for the same team,
   assignments, attempt, and frozen deadline before execution. Nearby numerical
   agreement alone is insufficient.
4. **Cancellation with an explicit timing contract.** Detect and disseminate
   withdrawals promptly. Budget time for cancellations and commitments to traverse
   the graph, and specify what happens when a fault occurs too late for everyone
   to learn about it before the deadline.
5. **Consistent execution and recovery.** Disseminate EXECUTED status. After partial
   execution, enter an explicit recovery/hold policy instead of allowing remaining
   agents to independently start later capture attempts. The physical recovery
   action needs its own design and validation.
6. **Match timeouts and lead time to the network.** Define supported latency/loss
   bounds; account for graph diameter and multiple negotiation exchanges. Review
   readiness freshness and state-neighbour staleness together. Longer timeouts
   also retain stale information longer, so they are not an unconditional remedy.
7. **Targeted verification around execution.** Supplement random tests with faults
   just before commitment and the deadline, including lost acknowledgements,
   partitions, and raw-perception loss. Preserve separate whole-team participation,
   attempt consistency, readiness, and synchronization criteria.

No finite acknowledgement exchange guarantees nontrivial all-or-none execution
under arbitrary unbounded message loss. A future protocol must state its network
and fault assumptions rather than claiming unconditional synchronized capture.
See [Reaching Agreement, Schneider et al.](https://www.cs.cornell.edu/info/people/fbs/publications/ReachingAgreement.pdf).

## Hardware observations that would motivate revisiting this work

For now, keep the 1-second working criterion and observe the existing implementation.
Record per-agent `capture_sync`, `current_mode`, `/readiness`, assignment/lock,
raw target timestamps, neighbour freshness, and clock synchronization evidence.
Measure capture-guidance activation separately if physical response timing matters;
a supervisor's published mode is not motor/contact confirmation.

Revisit the deferred work if hardware shows persistent spread beyond 1 s, partial
team activation, execution of different attempts, failure to resume after an outage,
or capture while a required member is unready. A hardware result under a shared
network does not establish correctness under restricted multi-hop communication.

## Evidence identifiers

Primary results and trace examples are recorded above so this document remains
useful if temporary files are cleaned up. At validation time, raw reports were in
`/tmp/consensus-benchmark-nominal150-20261113` and
`/tmp/consensus-benchmark-matrix-20261113`; these are temporary locations, not
repository-managed archives. A case can be regenerated using its size, topology,
profile, seed, duration, and matching source/model settings from the benchmark guide.

Recorded SHA-256 identifiers:

```text
nominal trials.csv: b50da74fd5c82005efee5481a617220295c142b58e315ffb295de7ad5953c12d
matrix trials.csv:  918ca50ba7ea84628767eacc21a0fb010a9db1eac761bc447afeff977378ac8a
consensus source:  b3b59a9cff23c244caaa117c81e02cc75398db12971055d41635479cdfc69c22
benchmark source:  a07f9629aaee5c1545d93477b7d811ab8db8ce7c6528649deb5aaf3bfde1fb54
```
