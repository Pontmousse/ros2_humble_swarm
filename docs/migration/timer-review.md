# Timer review: production per-robot stack (2026-11-26)

Scope: our container under `DRIVER_STACK=v2` with the production flags (KF, estimators, virtual
spacecraft on; viz off). Branch `split-containers`. Rates are the **resolved** values (launch dict >
`virtual_spacecraft.yaml` > code default). Message counts are derived from configured rates, not measured.

## The short version

- Timers themselves are cheap; **a timer that publishes is not**. Each publish is a serialize, a DDS
  send and a Python callback in every subscriber. The stack publishes **~510 timer-driven messages/s per
  robot**, and many carry nothing new.
- Some timers are load-bearing. **The landmark/target estimators' 15 Hz timer is the freshness
  mechanism**: it re-projects the marker memory through the live pose with a fresh stamp, so guidance never
  starves when the camera stalls. Same for pose_kf (20 Hz heartbeat), the auction/consensus 5 Hz rounds,
  and the UDP broadcast. **Those stay as they are.**
- The waste is elsewhere: topics nobody reads, rates above what the consumer samples, and controllers
  publishing constant zeros in modes where they are off. Fixing those is about **half the message
  traffic** with no change to what any controller sees.

## Where the messages go today (timer-driven, per robot)

| Source | Topics × rate | msg/s |
|---|---|---|
| virtual_spacecraft | `odom` + `applied_wrench` × 50 Hz | 100 |
| pose_kf | `pose` + `localization/odom` + `/tf` × 20 Hz | 60 |
| motion_planner (pg3) | `avoidance_wrench` + `flocking_wrench` + `target_position` × 20 Hz | 60 |
| contact_switch | `contact_detected` × 50 Hz | 50 |
| pointing_planner (pg2) | `pointing_wrench` + `target_orientation` × 20 Hz (+ `/target_estimate` 2 Hz) | 42 |
| state_broadcaster | `comms/neighbours` + `comms/nbh_pos` × 20 Hz (+ stats 1 Hz) | 41 |
| cp_controller | `capture_wrench` + `preparation_wrench` × 20 Hz | 40 |
| estimators ×2 | `landmarks`, `targets` × 15 Hz | 30 |
| velocity_smoother + lifecycle | `cmd_vel` 10 Hz + bond heartbeats 2 × 10 Hz + diag | 31 |
| nav2_pidff | `cmd_vel_raw` × 20 Hz | 20 |
| ap_auction, readiness | `/ap_bids` + `assigned_ap` + `assignment_locked`, `/readiness` + `consensus_debug` × 5 Hz | 25 |
| bounding_box_search | `spacecraft_wrench` × 10 Hz | 10 |
| run_config_snapshot | `run_config` (~100 KB) every 5 s | 0.2 |

## Tier 1: free wins, nothing downstream changes

**Status: done 2026-11-26 on `split-containers`.** Also removed `consensus_debug` and `target_position`
from `launch_rosbag.py`. A rosbag started 3 s after the single `run_config` publish still recorded it.

| # | Change | Where | Saves | Why it is safe |
|---|---|---|---|---|
| 1 | `run_config` publish once: `publish_period: 0.0` | launch `:220` (code skips the timer when ≤ 0) | 100 KB reliable burst every 5 s | Topic is TRANSIENT_LOCAL; the first publish serves late joiners. **Check one bag still contains `/RMx/run_config`** (campaign_extract needs it) |
| 2 | Drop `comms/nbh_pos` | `state_broadcaster.py:70` (`legacy_pub`) | 20 msg/s | No subscriber anywhere (legacy, the bag records `/RMx/nbh_pos`, a different name) |
| 3 | Drop `target_position` | `position_guidance3.py` publish loop | 20 msg/s | Only consumer is `ep_controller`, not launched |
| 4 | Drop `target_orientation` | `pointing_guidance2.py` | ≤ 20 msg/s | Only consumer is `pointing_controller`, not launched |
| 5 | `consensus_debug` off | `readiness_consensus_node.py` | 5 msg/s | Bag-only debug float; not read by the campaign scripts |

## Tier 2: cheap rate fixes, consumer still gets what it needs

| # | Change | Saves | Why it is safe | Watch |
|---|---|---|---|---|
| 6 | `contact_switch` `rate_hz` 50 → 20 (launch param) | 30 msg/s + 30 GPIO ioctl/s | Keeps the **level** stream (mode_supervisor latches only a `True` that arrives *in* CAPTURE, so publish-on-change would break DOCK). Debounce 3 samples = 150 ms | DOCK latency +90 ms; fine against the 5 Hz supervisor tick |
| 7 | `state_broadcaster` `receive_rate` 20 → 10 (= `broadcast_rate`) | 10 msg/s | Packets arrive at 10 Hz per neighbour; half of today's publishes carry only dead-reckoning. Closing speed ≤ 0.1 m/s → 1 cm per update | Avoidance updates at 10 Hz (`nbh_hold_timeout` 0.75 s) |
| 8 | `virtual_spacecraft` 50 → 20 Hz **and** `maximum_time_step` 0.05 → 0.1 | 60 msg/s (odom + applied_wrench) | nav2_pidff samples the reference at 20 Hz, so 50 Hz is 2.5× oversampled; integration is exact for constant wrench | Raise `maximum_time_step` in the same change or the reference runs slow. Validate HIL tracking error. `applied_wrench` stays (campaign_extract and bag_report read it) |

**Tier 1 + 2: ~165–185 msg/s less (~35%), plus the 5-second 100 KB burst.**

**Status: Tier 2 done 2026-11-26.** `contact_switch` default `rate_hz` 20 (covers `launch_contact_switch.py`
too); `receive_rate` 10 in both launches; `virtual_spacecraft` `timer_frequency` 0.05 with
`maximum_time_step` 0.1. In SIL the reference publishes at ~19 Hz and `comms/neighbours` at 10 Hz.

## Tier 3: mode-idle zeros (bigger gain, needs one rule)

In SEARCH (the longest phase) these publish constant zeros at full rate:

| Publisher | Zero when | msg/s in SEARCH |
|---|---|---|
| cp_controller `capture_wrench`, `preparation_wrench` | outside CAPTURE / outside locked ENCAP | 40 |
| motion_planner `flocking_wrench` | outside ENCAP | 20 |
| motion_planner `avoidance_wrench` | no neighbour within range | up to 20 |
| pointing_planner `pointing_wrench` | CAPTURE, DOCK | (0 in SEARCH) |
| bounding_box_search `spacecraft_wrench` | outside SEARCH, before arm | (0 in SEARCH) |
| nav2_pidff `cmd_vel_raw` | before arm / inputs stale | 20 (pre-arm only) |

**Rule:** publish while non-zero, publish **one explicit zero on the falling edge**, then stop.
virtual_spacecraft holds a source's last value for `wrench_timeout` (0.25 s) and then drops it
(= zero), so silence after an explicit zero is exactly today's behaviour. A **non-zero** output must keep
streaming faster than 4 Hz. The topics are RELIABLE, so the single zero is delivered.

Also stop the **callback work** in modes where the output is off: cp_controller's `target_callback`
(least-squares velocity fit, the heaviest guidance callback) runs in SEARCH; flocking potential runs
outside ENCAP. Start the target track at ENCAP entry, since it needs its 5.25 s warm-up before CAPTURE.

**Cost:** bags show gaps instead of zeros. `campaign_extract.py` reads `\w+_wrench`, so it must
treat a gap the way virtual_spacecraft does (hold 0.25 s, then zero). **Saves ~60–80 msg/s in SEARCH.**

**Status: Tier 3 done 2026-11-26.**
- **Gated publishers.** `swarm_controller/zero_gate.py` (`ZeroGatedPublisher`) wraps the six wrench
  publishers: `spacecraft_wrench`, `avoidance_wrench`, `flocking_wrench`, `pointing_wrench`,
  `capture_wrench`, `preparation_wrench`. `test_zero_gate.py` checks that virtual_spacecraft's
  summed input is identical with and without the gate.
- **Flocking work.** The potential is computed only in ENCAPSULATION.
- **Extractor.** `campaign_extract.py` samples the layers time-weighted on its grid, with the same hold rule.
- **Not gated on purpose:**
  - nav2_pidff's zero `cmd_vel_raw`, because it is the stop-the-chassis path.
  - cp_controller's `target_callback`, because the 5.25 s target track must be warm when CAPTURE starts and ENCAP can be short.
- **One SIL run:** N=2, QC OK, 2/2 docked in 48.3 s, `trk_p95` 23 mm, consensus sync 75 ms. Each layer is
  active only in its phase; each wrench topic carries ~470 messages instead of ~1,220 over the 61 s run.

## Do not touch: rates that consumers depend on

| Timer | Rate | Who depends on it |
|---|---|---|
| landmark/target estimators | 15 Hz | **This is the marker memory doing its job.** Guidance drops to damp-only after 0.3 s without `landmarks`; ap_auction stops bidding after 0.4 s; mode_supervisor counts 3 frames; cp_controller's capture law runs once per `targets` message and **has no expiry if the topic stops** |
| pose_kf | 20 Hz | virtual_spacecraft freezes at 0.25 s, nav2_pidff stops the chassis at 0.3 s, state_broadcaster drops the robot at 0.5 s; predict `dt` clamps at 0.2 s |
| ap_auction, readiness_consensus | 5 Hz | Peer timeouts 0.5 s on sender stamps (`/ap_bids`, `/readiness` are heartbeats); cp_controller needs `assigned_ap`/`assignment_locked` younger than 0.5 s |
| state_broadcaster TX | 10 Hz | `stale_timeout` 0.5 s sets `valid`, which drives auction and consensus membership |
| velocity_smoother | 10 Hz | pose_kf treats `cmd_vel` older than 0.25 s as zero in its prediction |
| pointing_planner | 20 Hz | torque slew `dt` already clamped at 0.05 |
| any non-zero wrench | ≥ 4 Hz | virtual_spacecraft `wrench_timeout` 0.25 s |

These go away only by **merging** (the timer is replaced by a function call in the same tick), never by slowing
them down. See `nodes-to-functions.md`.

## Tier 4: found along the way (not timers)

Rule: a new timeout fires only on a **dead** source, never on jitter. With healthy inputs only #1 and #5 change anything.

**Status: done 2026-11-26** (except #4 and #7, decided as below).

| # | Issue | Fix |
|---|---|---|
| 1 | mode_supervisor `wait_for_service(0.5)` on each transition; if the service was not ready the call was **dropped** while `mode` still advanced, so a layer stayed off (the ~1/10 SIL "`encapsulation/enable` not available" failures) | `call()` records the wanted state; `flush()` (every tick) sends it without blocking and resends until a `success` reply: 1 s, 4 s for `engage_wheels` (v2 ack up to 3 s). Latest state wins, call order kept. `test_mode_supervisor_calls.py` |
| 2 | cp_controller held its last `targets`-derived command forever if `targets` stopped | the capture command is active only while `last_target_time` is younger than `target_hold_timeout` (0.4 s); then one zero (zero gate) and silence. `test_capture_approach.py` |
| 3 | pose_kf dead-reckoned and published forever with every input dead; `gps/odometry/gyro_timeout` unread | output withheld while wheel odom is silent > `odometry_timeout` (now 1.0 s in both launches; prediction still bridges shorter gaps), so the downstream freeze/stop guards fire. `gps_timeout`, `gyro_timeout` removed. `test_pose_kf_stale.py` |
| 4 | Estimators never forget, so `lost_timeout` never fires | **kept on purpose** (marker memory guarantee); launch comment says `lost_timeout` is inert. Revisit with moving-target work |
| 5 | Capture fired on the first 5 Hz tick after `armed_time`, then the supervisor's next 5 Hz tick (≤ 0.4 s skew) | readiness schedules a one-shot round at `armed_time` (cancelled on round reset); supervisor ticks immediately on `capture_sync`. `test_readiness_consensus.py` |
| 6 | `launch_virtual_spacecraft.py` not bind-mounted | copied and mounted `:ro` like the decentralized launch (`vars.yml`, `copy_config.yml`, `container.yml`) |
| 7 | `maximum_force` 0.02 N vs contributor caps 0.5–1.0 N: any non-zero source saturates the sum | **unchanged**; comment in the yaml. Rescaling caps and gains into the budget is a separate retune (SIL campaign + HIL) |

**SIL (N=2, foreground, 3 runs, same host):**
- Capture skew: Tier 3 fired 126/201 ms after `armed_time` and switched mode at 286/331 ms. Tier 4 fires at +1/+4 ms and switches at +1/+8 ms; `sync` is 2 ms in both Tier 4 runs (Tier 3: 75 ms).
- No retry or withheld-pose warnings in any run; both robots docked in all runs.
- Run-level `trk95` 46 and 82 mm (Tier 4) vs 88 mm for the Tier 3 runtime rebuilt and run right after on the same loaded
  host (load avg ~6). It is dominated by a 1–2 s transient at mission start in both builds, so it is host drift, not Tier 4.
- **Open:** RM2's ENCAP-phase p95 was 30/58 mm in the two Tier 4 runs vs 16/22 mm in the two Tier 3 runs. No Tier 4
  code path changes ENCAP guidance on a healthy run (the only one active there is the readiness one-shot, which moves
  consensus timing), but 2 vs 2 runs cannot settle it. Needs a proper A/B before HIL.

## How to measure

Before and after each tier, on an armed robot (`vcgencmd get_throttled` = `0x0`):

```bash
pidstat -u 1 60
```

Plus the HIL bag metrics that define "no performance loss":
- `trk_p95`, the tracking error, from `campaign_extract.py`;
- landmark/target age at the consumers;
- mode transition times and dock success.
