# Migration plan: nodes → function calls (fewer processes, less DDS)

Branch `split-containers`. Companion to `timer-review.md` (rates, consumers, freshness) and
`docs/contracts/driver-contract-v100.md` (what the drivers container gives us).

## Goal and the one constraint

Today our container runs **19 processes** per robot (~20 DDS participants per Pi with the drivers),
and every robot's participants discover each other. Target: **5 processes**, same behaviour.

**Constraint: no loss of robot performance.** Freshness must not regress. The marker memory exists so
controllers always see a fresh landmark/target cloud even when the camera stalls; any merge must keep
that guarantee, and must keep every "do not touch" rate in `timer-review.md` (heartbeats, timeouts),
either as a topic at the same rate or as a function call in the same tick.

## End state

| Process | Replaces (today's node #) | Driven by |
|---|---|---|
| **perception** | gnss_bridge (1), aruco_bridge (2), pose_kf (3), landmark_estimator (4), target_estimator (5) | one 20 Hz tick (today's pose_kf tick) + input callbacks |
| **guidance** | motion_planner (8), pointing_planner (9), cp_controller (13) | input callbacks + one 20 Hz tick |
| **control** | virtual_spacecraft (6), bounding_box_search (7), nav2_pidff (10), velocity_smoother + lifecycle (11, 12) | `localization/odom` arrival (20 Hz) |
| **swarm** | mode_supervisor (14), ap_auction (15), readiness_consensus (16), contact_switch (18), run_config_snapshot (19) | one 5 Hz tick + events |
| **comms** | state_broadcaster (17) | its UDP timers (unchanged) |

Five processes, not one: Python's GIL runs one process on one core, so the groups keep using the
Pi's 4 cores. Groups follow data flow, so the topics that remain between them are the ones you
want in a bag anyway.

## Rules for every phase

- **ROS-free core, thin node, one module per job.** Algorithms move into plain modules (functions/classes,
  ROS-free where possible), like `swarm_target/estimators.py`, `marker_memory.py`, `pose_history.py`,
  `swarm_controller/avoidance.py`, `aruco_convert.convert`. The merged node file only declares parameters
  and wires callbacks, timers and publishers to those modules; **no single huge node script**. Each phase
  plan carries a module-layout table (module → job → origin). The modules are unit-tested without ROS
  (see `test_shared_target.py`, `test_aruco_convert.py`, `test_perception.py`).
- **Old path deleted, not switched.** On `split-containers` a merged phase replaces the old nodes and their
  launch blocks outright; the `hil` branch is the rollback.
- **Outward contract unchanged per phase.** Topics consumed by nodes outside the merged group keep their
  name, type, QoS and **rate**. Internal topics disappear, unless `debug_topics:=true` republishes them
  for a bag.
- **Each phase in two steps.**
  - **a. Co-locate:** run the existing nodes unchanged in one process with one executor. Humble gives one
    participant per process, so this removes processes and discovery traffic with almost no code risk.
    Messages between them are still serialized.
  - **b. Fuse:** replace the internal topics and timers with direct calls.
  - Only one slow callback can starve the others in step a, so audit blocking calls first
    (e.g. `mode_supervisor`'s `wait_for_service(0.5)`).
- **Simulator stays green.** `fake_plant` and `virtual_scene` feed the legacy topics (`mm_pos_unf`,
  `landmarks_unf`, `targets_unf`). Merged nodes accept both input formats (`input_format: legacy | v2`).
- **Both launch files mirror each other** (AGENTS.md).
- **Acceptance per phase** (SIL first, then one robot in HIL):
  - same `trk_p95`;
  - same landmark/target age at the consumers;
  - same mode transition times and dock success;
  - plus `pidstat` CPU before/after.

## Phase 0: baseline (½ day)

- On one armed robot: `pidstat -u 1 60` per process, `vcgencmd get_throttled`, and a HIL bag with
  `campaign_extract.py` metrics.
- Same in SIL with N=3.
- These numbers are the bar every later phase is measured against.

## Phase 1: timer quick wins (1 day)

Tiers 1 and 2 of `timer-review.md`:
- `run_config` once;
- drop `comms/nbh_pos`, `target_position`, `target_orientation` and `consensus_debug`;
- `contact_switch` at 20 Hz;
- `receive_rate` 10;
- virtual_spacecraft at 20 Hz with `maximum_time_step` 0.1.

Mostly parameters, all reversible. **~165 msg/s less per robot.**

## Phase 2: perception (#1–5), first priority

**Status: done 2026-11-26 on `split-containers`** (steps 2b–2d in one go; 2a skipped because launch params and remaps
apply process-wide). One `perception` process per robot replaces pose_kf, both estimators and both bridges. The old
nodes, the `DRIVER_STACK`/`KF_LOCALIZATION`/`TARGET_ESTIMATOR` switches and the robomaster-stack launch blocks are deleted.

| Module | Job |
|---|---|
| `swarm_filter/pose_kf.py` | KF base node, unchanged maths (no executable of its own) |
| `swarm_target/marker_memory.py` | `MarkerMemory`: frozen-stamp gate, `lift`, `project`, `memory_viz` (moved verbatim from the estimator node) |
| `swarm_target/pose_history.py` | pose ring buffer for `lift_at_stamp` (default off until a HIL A/B) |
| `swarm_aruco/aruco_convert.py`, `swarm_marvelmind/gnss_convert.py` | the former bridges' pure conversions |
| `swarm_target/perception_node.py` | thin node: inputs per `position_input`/`marker_input` (`v2` or `legacy`), projection in the KF's `_publish` |

**Checked:**
- unit tests (`test_perception.py` and others);
- launch node lists for the Pi, SIL and HIL with a virtual target;
- live v2 inputs on an isolated domain: `pose`, `landmarks` and `targets` at 20 Hz, `targets_unf` at the camera rate;
- SIL N=2: QC OK, 2/2 docked, per-phase `trk_p95` 18–34 mm, capture sync 3 ms.

**CPU** (SIL, both robots):
- `perception` uses 269% of a core against 277% for pose_kf plus four estimators. The merge alone barely moves CPU on the dev PC.
- The real cost was OpenBLAS threads spinning in the KF: capped at one thread (as the Pi already is, `lib_threads`),
  `perception` drops to 43%, and the whole stack from 457% to 238%. `sil_batch.py` now sets the caps.

**Not yet:**
- HIL on one robot with the rebuilt image;
- the `lift_at_stamp` A/B;
- `pidstat` on a Pi.

**Why first:**
- It is where the freshness guarantee lives.
- It has the most internal hops: `mm_pos_unf`, `landmarks_unf`, `targets_unf`, and `pose` into both estimators.
- It has the most duplicated work: the two estimators run identical code on identical timers.
- Merging it also fixes a real hazard: today the estimators project through whatever `pose` they last
  received, with no age check.

**Design.** One node, one 20 Hz tick:
```
on odom / gnss fix / cmd_vel   -> KF update (today's pose_kf callbacks, unchanged maths)
on aruco frame (v2) or Markers (legacy)
                               -> convert (aruco_bridge.convert) -> lift with the pose at the
                                  capture stamp (small pose ring buffer) -> memory.observe
                                  (today it uses the pose at receive time)
tick (20 Hz):  KF predict -> publish pose, localization/odom
               -> project landmark memory and target memory through THIS pose -> publish landmarks, targets
```

**What this keeps:**
- `landmarks`/`targets` still come out on a fixed clock with a fresh stamp, now at 20 Hz instead of 15 Hz,
  and from the **same** pose that was just published. The memory still keeps feeding guidance when the
  camera stalls.
- All consumer timeouts (0.3 s guidance, 0.4 s auction) get more margin, not less.
- mode_supervisor's `detect_frames: 3` becomes 0.15 s instead of 0.2 s.
- `targets_unf` is still published per camera frame, because readiness_consensus uses it for raw
  liveness.
- `mm_pos_unf` and `landmarks_unf` are republished only with `debug_topics`.

**Steps:**
1. 2a: co-locate the 5 nodes in one process.
2. 2b: one estimator node holding both memories (removes a timer and a duplicate `pose` stream).
3. 2c: drive the projection from the KF tick (removes the estimator timer and the `pose` hop).
4. 2d: call `convert`/`to_hedge` directly on the drivers' topics. The bridge nodes disappear.

**Removes:** 4 processes, 1 timer, ~4 internal topics. The drop in timer-driven messages is small,
because the outputs are kept at the same rate on purpose. The win is in processes, hops and DDS participants.

**Risk:**
- One crash takes out localization and perception together. They already fail together today,
  since the estimators are useless without `pose`.
- Respawn costs the marker memory. Optionally persist the memory to `/tmp` and reload it on start.

## Phase 3: wrench contributors (#8, #9, #13), second priority

**Why:**
- All three read the same inputs (`pose`, `localization/odom`, `landmarks` or `targets`,
  `current_mode`), so `landmarks` is delivered twice and `targets` into two of them.
- Each republishes cached values on its own 20 Hz timer.
- In SEARCH most of their output is constant zero (`timer-review.md` tier 3).

**Design.** One guidance node:
- Each input has one subscription; the current maths runs in the input callbacks (flocking and
  neighbour repulsion; pointing centroid and share estimate; capture-law track).
- One 20 Hz tick publishes the wrench topics.
- **Mode-gated:**
  - flocking is computed only in ENCAP;
  - the capture track starts at ENCAP entry (5.25 s warm-up before CAPTURE);
  - each zero output gets one falling-edge zero, then silence.
- `/target_estimate` stays at 2 Hz.

**What this keeps:**
- virtual_spacecraft's wrench contract, until Phase 4: the same `*_wrench` topics, and any non-zero
  output streamed at 20 Hz (> 4 Hz needed).
- The pointing slew limiter's 20 Hz tick.
- The distinction between velocity sources: pointing and flocking damp on the **physical**
  `localization/odom`, while capture servos on the **virtual** craft.

**Steps:**
1. 3a: co-locate.
2. 3b: shared input cache + one tick.
3. 3c: mode gating, together with the `campaign_extract.py` change (a gap in a `*_wrench` topic = hold 0.25 s, then zero).

**Removes:** 2 processes, 2 timers, ~60–80 msg/s in SEARCH, and the heaviest callback (cp's
least-squares velocity fit) outside ENCAP/CAPTURE.

## Phase 4: control chain (#6, #7, #10–12)

**Why:** the timers here are stacked in series. pose_kf 20 Hz → virtual_spacecraft 50 Hz → pidff 20 Hz →
smoother 10 Hz gives **~200 ms worst-case feedback latency, half of it from the smoother**. pidff output is
half discarded, and the smoother's bond has already died on a starved Pi (2026-10-09).

**Design.** One control node, triggered by each `localization/odom` (20 Hz):

```
sum the wrench inputs (same wrench_timeout hold rule)
  -> integrate the virtual craft (dynamics.py, exact)
  -> bounding-box force for the next step
  -> PID+FF tracking
  -> accel/velocity limits (Python port of the smoother's clamp, inside the PID's anti-windup)
  -> publish cmd_vel
```

`cmd_vel` stays a 20 Hz stream: the drivers chassis stops after 0.5 s, and pose_kf's prediction
needs it fresher than 0.25 s.

**Removes:**
- 4 processes, including a C++ node and the lifecycle manager with its bond heartbeats;
- 3 timers;
- ~100+ msg/s;
- **~150 ms of worst-case feedback latency.**

**Risk:** highest of all phases. This is the validated HIL loop.
- Re-check virtual-vs-physical tracking error on the 2026-09-09 envelope (1.0 N / 0.2 Nm).
- Keep the old chain selectable (`CONTROL_CHAIN=nodes|fused`) until two campaigns match.

## Phase 5: swarm logic (#14–16, #18, #19)

**Design:**
- auction and consensus run on the same 5 Hz tick, and consensus reads the auction's result directly
  (the `assigned_ap`/`assignment_locked` hop goes away internally).
- `/ap_bids` and `/readiness` stay 5 Hz heartbeats, because peers time out at 0.5 s.
- `assigned_ap`/`assignment_locked` stay 5 Hz while cp_controller (in guidance) relies on its 0.5 s
  `assignment_timeout`.
- mode_supervisor joins the same process:
  - asynchronous service calls instead of `wait_for_service` in the timer;
  - a one-shot timer at `armed_time` instead of the next tick (removes up to ~0.4 s capture skew).
- contact switch becomes gpiod edge events inside the supervisor, plus a level republish while
  pressed (DOCK latch rule).
- `run_config` is published once, latched, from the same process.

**Removes:** 4 processes, 2 timers, ~60 msg/s (with the contact stream).

## Phase 6 (optional): comms

state_broadcaster can stay on its own: socket work, a different failure mode, small.
- Publish `comms/neighbours` on packet arrival instead of a 20 Hz poll (`select` on the socket).
- Or fold it into the swarm process.

## Order and size

| Phase | Effort | Processes after | Risk | Main gain |
|---|---|---|---|---|
| 0 baseline | ½ day | 19 | none | the bar |
| 1 timers | 1 day | 19 | low | ~165 msg/s, 100 KB/5 s |
| 2 perception | 3–4 days | 15 | medium | 4 processes, the pose/estimator hazard gone, capture-time lifting |
| 3 guidance | 3–4 days | 13 | medium | 2 processes, SEARCH zeros, heavy callbacks gated |
| 4 control | 4–5 days | 9 | **high** | 4 processes, −150 ms feedback latency, no bond |
| 5 swarm | 2–3 days | 5 | low–medium | 4 processes, capture skew fix |

Each phase ships on its own and is reverted by a launch flag. Measure CPU after each one: if Phases 1–3
already put the Pi comfortably under budget, Phase 4 can wait for its own campaign.
