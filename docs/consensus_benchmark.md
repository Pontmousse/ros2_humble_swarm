# Consensus synchronization benchmark

This is a **software coordination simulation** of the existing readiness-consensus
and mode-supervisor code. It does not initialize ROS, join DDS, open sockets, or
command robots. ROS Python libraries and built `swarm_interfaces` messages are
still required for the real message types and ROS time arithmetic.

The runner tests the current algorithm, including its limitations. It does not
add a capture-team roster, multi-hop forwarding, or stronger commitment rules.

See [recorded findings and deferred improvements](consensus_findings.md) for the
2026-11-13 evaluation, the 1-second working hardware criterion, observed failure
traces, and the decision to defer protocol changes pending hardware observations.

## Running

After building and sourcing the workspace, use the installed executable:

```bash
ros2 run swarm_controller consensus_benchmark \
  --repo-root /home/elghali/projects/ros2_humble_swarm \
  --num-seeds 150 --sizes 2 3 4 5 6 7 8 \
  --topologies complete --profiles nominal \
  --output-dir /tmp/consensus-nominal-150
```

This runs 1,050 trials: 150 seeds for each size, with the complete communication
graph and nominal latency. All trials continue for 30 simulated seconds, including
after the first robot captures. A nonzero synchronization failure rate is a result,
not a script error.

To run directly from the source tree without reinstalling `swarm_controller`:

```bash
PYTHONPATH="$PWD/ros2_swarm/src/swarm_controller:$PWD/ros2_swarm/src/swarm_comms:$PYTHONPATH" \
  python3 -m swarm_controller.consensus_benchmark \
  --num-seeds 1 --output-dir /tmp/consensus-matrix-smoke
```

The default matrix covers 7 sizes × 5 topologies × 9 profiles. `--num-seeds`
means seeds **per matrix cell**, not a single seed value or a total batch size.
The default is 150 (47,250 trials); 1,000 means 315,000 trials. First use a filtered
batch or the one-seed smoke matrix to estimate runtime and output volume.
Failure timelines can consume substantial space.

```bash
# One reproducible case, using seed 1000; include its timeline even if it succeeds.
ros2 run swarm_controller consensus_benchmark \
  --num-seeds 1 --seed-start 1000 --sizes 3 \
  --topologies chain --profiles nominal --trace \
  --output-dir /tmp/consensus-chain-seed1000

# Focus on larger swarms and network faults.
ros2 run swarm_controller consensus_benchmark \
  --num-seeds 150 --sizes 6 7 8 --topologies chain ring random \
  --profiles latency loss partition --output-dir /tmp/consensus-network-stress

# Reproduce a saved failure, including its original model settings.
ros2 run swarm_controller consensus_benchmark \
  --replay /tmp/consensus-network-stress/traces/6-chain-loss-0.json \
  --output-dir /tmp/consensus-replay
```

Output directories must be new, preventing accidental mixing of batches.
Use `--help` for model settings. Examples include `--duration 60`,
`--sync-tolerance-ms 100`, `--delay-min-ms 60 --delay-max-ms 100`, and
`--local-delay-ms 2`. Model overrides change synthetic inputs, not production
algorithm parameters. Replaying a saved trace uses its saved settings; source or
production-parameter mismatches are rejected.

If an installed module differs from this repository, rebuild/source the matching
workspace or use the source invocation above. Changes to `Readiness.msg` require
rebuilding `swarm_interfaces`. `--repo-root` is necessary when the checkout cannot
be found from the module path or current directory.

## What is reproduced

An offline I/O adapter invokes the production node constructors, registers their
subscriptions and timers, and executes their actual callbacks. It replaces ROS
clocks, transport, logging, and service endpoints. It does not override
`local_ready`, consensus, arming, firing, detection gates, or mode decisions.
The ROS base constructor is temporarily bypassed during offline construction;
this runner must run in its own process, not inside a live ROS executor.

The launch files are parsed without executing them. Consensus, supervisor, and
state-broadcaster parameter blocks must agree between centralized and decentralized
launches. Applicable spacecraft YAML overrides are considered. Unsupported blocks
or stale loaded production modules cause an explicit error. The manifest records
resolved parameters and hashes of the source, launch, YAML, and message files.

Each agent has an independent simulated clock and independently phased periodic
timers. The event queue uses integer nanoseconds and seeded ordering for simultaneous
events. Each link/channel has its own random stream. Clock offsets affect the
production clock and message stamps; reported execution times use the common
reference clock. The reference epoch starts at ROS time 100 seconds so the
production zero-timestamp sentinel remains meaningful.

Neighbour state is sent at the configured broadcast rate and drained at the
configured receive rate. The production packet encoder/decoder and `NeighbourTable`
handle packets and freshness. State poses are always valid; odometry and physical
motion are not simulated. Expired entries become invalid naturally. Readiness and
state messages traverse only graph edges; local topics have a separate delivery
delay, zero by default. There is no implicit routing through intermediate agents.
ROS QoS queueing and retransmission behavior are not simulated.

Default synthetic inputs:

| Input | Distribution / behavior |
|---|---|
| Assignment availability | Independent uniform 0–4 s per robot |
| Assignment and lock | Unique assigned ID and locked status, refreshed at 5 Hz |
| Raw observations | 10 Hz, newly advancing stamps, own ID visible once available |
| Estimated targets | 15 Hz, own ID at 0.6 m once available |
| Landmarks | Present at the estimated-target rate |
| Contact and manual overrides | None |
| Local delivery | Zero delay, no loss |
| Capture services | Available; requests accepted immediately |

These inputs replace auction/perception outputs, not the production checks of those
outputs. Estimated targets remain available during raw-perception interruptions.
This isolates coordination rather than attempting to reproduce a physical scene.

## Matrix and fault profiles

Topologies are `complete`, `chain`, `ring`, `star`, and `random`. Random graphs
start with a random spanning tree and add each extra possible edge with probability
0.3. Graphs are undirected and initially connected; small sizes can produce
identical graphs in different topology families, reported separately.

| Profile | Additional conditions |
|---|---|
| `reference` | Zero network delay; timer phases still independent |
| `nominal` | Independent uniform 53.6–103.6 ms network delay |
| `latency` | Independent uniform 0–600 ms delay, allowing reordering |
| `loss` | Per-trial drop probability uniform 0–20%; independent per transmitted packet |
| `perception` | One robot stops raw observations for 0.2–1.5 s |
| `frozen` | One robot keeps publishing raw observations with its last timestamp for 0.2–1.5 s |
| `clock` | Independent fixed clock offsets uniform ±50 ms |
| `never_ready` | One robot remains unassigned/unlocked with no visible target |
| `partition` | A random two-sided cut blocks transmissions in both directions for 1–3 s |

Except for `reference` and `latency`, profiles use nominal latency. Temporary fault
start times are uniform 0–8 s; the affected robot/cut is seeded. A partition blocks
transmission while active; already in-flight packets may arrive. Each profile
isolates one fault family rather than combining arbitrary faults. Some faults occur
before readiness or after capture; this is part of the stated sampling distribution.

The 78.6 ms delay centre comes from the repository's documented state-transport
measurement in `swarm_comms/table.py`. The chosen uniform distribution and its use
for readiness traffic are approximations, not a measured DDS model. Clock offsets
are constant during a trial; clock drift and OS scheduling stalls are not modeled.

## Success and reports

The required roster always contains the entire configured swarm, even if production
neighbour lists shrink. The first CAPTURE entry for each robot determines:

- `synchronized_capture`: every robot enters, with spread **≤200 ms** by default.
- `unsynchronized_capture`: every robot enters, with a larger spread.
- `partial_capture`: only some robots enter before the horizon.
- `no_capture`: nobody enters before the horizon.
- `execution_error`: the simulator could not execute the case; counted explicitly.

Synchronization and readiness safety are separate. A synchronized run can still
be flagged unsafe if any required robot was locally unready when a trigger or
CAPTURE transition happened. Other flags identify duplicate triggers, differing
firing rounds, and differing frozen deadlines. Deadline equality is reported
exactly, independently of the production 50 ms agreement tolerance.

Never-ready cases also report restraint: nobody triggered or entered CAPTURE.
Restraint is not a synchronization success. Normal-operation (`reference`, `nominal`)
and stress rates are reported separately; no blended headline rate is produced.

Outputs:

- `manifest.json`: source hashes, parameters, distributions, seeds, and scope.
- `trials.csv`: one row per case, outcome, trigger/mode spread, firing fraction,
  first/full capture times, highest round, and safety flags. Time columns use
  integer nanoseconds from trial start; spreads among a partial group remain
  diagnostic and cannot make that case pass.
- `summary.json` / `summary.md`: counts, rates, 95% Wilson intervals, unsafe and
  restrained counts, grouped by normal/stress, size, and matrix cell. Execution
  errors remain in the denominator.
- `traces/*.json`: replayable unsuccessful or unsafe cases, with per-agent input
  settings and event timelines. `--trace` includes synchronized successes too.

A finished batch exits 0 even when coordination fails; it exits 1 for case execution
errors and 2 for configuration/output/replay errors. Progress includes trial count
and runtime. Statistical intervals characterize only the specified sampling model;
replay and case outcomes do not depend on batch ordering.

## Interpretation limits

Capture-mode time is the supervisor's published transition, with an immediately
available service endpoint. It is not controller acknowledgement, motor activation,
or physical contact. Independent 5 Hz consensus and supervisor phases can create
spread even when proposed deadlines agree. The runner must expose this behavior,
not align timers or relax the 200 ms threshold to improve results.

No auction, actual cameras, robot dynamics, DDS middleware scheduling, hardware,
or orbital dynamics are simulated. Successful randomized cases are evidence about
this software model, not a proof of distributed correctness or a hardware timing
guarantee. Failures should be replayed before changing the production protocol.
