# Nav2 integration: four-step experiment

Use Nav2 per robot as a path-tracking layer. Keep formation, landmark attraction,
neighbor repulsion, and capture decisions in the existing swarm layer. Validate
each step on one robot before extending it to the swarm.

## 1. Prepare localization and TF

**Why:** Nav2 locates and controls the robot through TF. The current custom
`StatePos` is useful to the swarm code, but it does not provide Nav2's required
continuous, timestamped global-to-local transform.

**Architectural changes:** Extend the localization layer (or add a small TF
bridge beside it) to fuse Marvelmind/global pose with driver odometry, publish
`swarm_map -> RMx/odom`, and keep `StatePos` as a compatibility output. Add the
node and frame parameters to each robot's bringup launch file.

Standardize every robot on this TF chain:

```text
swarm_map -> RMx/odom -> RMx/base_link
```

- Keep the RoboMaster driver as the sole publisher of `RMx/odom ->
  RMx/base_link` and of `RMx/odom`.
- Add a localization node that combines the Marvelmind global position with the
  robot odometry and publishes only `swarm_map -> RMx/odom`.
- Compute that correction from synchronized poses:
  `T_swarm_map_odom = T_swarm_map_base * inverse(T_odom_base)`. Do not publish
  the Marvelmind position directly as an `odom -> base_link` transform.
- Use SI units, radians, ROS timestamps, meaningful covariance, and unique
  namespaced frame IDs. Confirm the yaw convention and axis directions.
- Continue publishing `StatePos` temporarily for legacy nodes, but derive it
  from the same state estimate so the legacy and Nav2 paths agree.

Acceptance checks:

- `tf2_echo swarm_map RM1/base_link` is continuous while the robot moves.
- TF has no duplicate publishers, jumps, stale timestamps, or frame cycles.
- The Nav2 transform tolerance is larger than measured localization latency,
  without hiding long delays.

## 2. Add mode control and velocity arbitration

**Why:** Teleoperation, capture, legacy control, and Nav2 can all generate
velocity commands. Without one arbiter, concurrent publishers can overwrite one
another unpredictably; a BT alone does not enforce exclusive topic ownership.

**Architectural changes:** Remap every controller to a dedicated command topic,
add one namespaced `twist_mux` plus priority/timeout configuration per robot,
and make it the only upstream source for the smoother. Add a small mode manager
first; later, a BT can call the same actions/services to sequence mission modes.

A behavior tree or small mission manager should choose the operating mode; it
should not be the final safety mechanism. A timeout-based velocity mux must be
the only component allowed to feed the velocity smoother and driver.

```text
teleop ---------------> cmd_vel_teleop --+
capture controller ---> cmd_vel_capture --+--> twist_mux
Nav2 controller ------> cmd_vel_nav -------+       |
legacy controller ----> cmd_vel_legacy ----+       v
                                             cmd_vel_selected
```

Suggested priority is emergency stop, teleoperation, capture, Nav2, then legacy
control. Give every input a short timeout so a dead publisher loses ownership.
On a mode transition, cancel the active action, wait for or command zero
velocity, then enable the next source. Also configure a nonzero RoboMaster
driver command timeout.

Start with explicit mode services/actions and status reporting. Add a BT when
multi-step behavior is needed, for example:

```text
Search -> SwarmFollowPath -> VisualCapture -> ReturnToTeleop
```

The BT owns sequencing, cancellation, success, and failure handling. The mux
always owns command arbitration.

## 3. Track swarm-generated paths with DWB

**Why:** The current controller chases a target that may change every 10 ms.
Separating slower swarm decisions from faster path tracking should produce more
stable motion while preserving the existing attraction and repulsion policy.

**Architectural changes:** Add a `swarm_path_adapter` node/action client, Nav2
message dependencies, one controller-server configuration per robot, and launch
entries for the controller server and its lifecycle management. The adapter
converts guidance output into short paths; DWB replaces only the legacy P
controller in Nav2 mode, not the swarm guidance.

Run `nav2_controller`'s `controller_server` in each robot namespace and use its
`nav2_msgs/action/FollowPath` action with
`dwb_core::DWBLocalPlanner` as the first holonomic controller candidate.

```text
position_guidance3
  landmark attraction + neighbor repulsion
             |
             v
swarm_path_adapter
  CoordXY/potential output -> stamped nav_msgs/Path in swarm_map
             |
             v
RM1/controller_server (FollowPath + DWB)
             |
             v
RM1/cmd_vel_nav -> mux -> velocity smoother -> driver
```

The adapter should publish/send a short receding-horizon path rather than a new
`NavigateToPose` request every 10 ms. Each `PoseStamped` must have a valid
timestamp, `swarm_map` frame, and normalized quaternion. Update the path at a
slower policy rate (start around 5-10 Hz); let the controller track it at a
stable higher rate (start around 20 Hz). Reject paths when localization,
landmarks, or neighbor data is stale.

Preserve `pointing_guidance2` initially by encoding its desired yaw in path
poses. If DWB cannot translate laterally while maintaining that yaw, evaluate a
task-specific orientation critic/controller rather than weakening the swarm
requirement silently. Configure nonzero `linear.y` velocity and acceleration
samples for the mecanum base.

The controller server still requires a local costmap, goal checker, progress
checker, footprint, odometry, and TF even when testing in an open area. First
compare Nav2 and the legacy P controller on the same short paths, measuring
cross-track error, completion time, oscillation, stops, and action failures.

## 4. Add and tune the velocity smoother

**Why:** Saturating velocity limits abrupt speed, but it does not limit how
quickly commands change. Smoothing reduces jerky motion and stopping variability
and gives all command modes the same measured chassis constraints.

**Architectural changes:** Add a namespaced velocity-smoother YAML section and
launch node after the mux, remap its output to the driver's existing `cmd_vel`,
and enable a nonzero driver watchdog. No swarm algorithm changes are required.

Place one smoother after the mux so teleop, capture, Nav2, and legacy commands
all receive the same physical limits:

```text
cmd_vel_selected -> velocity_smoother -> cmd_vel -> RoboMaster driver
```

Initial, deliberately conservative configuration:

```yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 50.0
    feedback: "OPEN_LOOP"
    scale_velocities: true
    max_velocity: [0.50, 0.50, 1.00]
    min_velocity: [-0.50, -0.50, -1.00]
    max_accel: [0.30, 0.30, 0.80]
    max_decel: [-0.50, -0.50, -1.20]
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 0.25
    odom_topic: "odom"
    odom_duration: 0.1
```

Remap the smoother's input `cmd_vel` to `cmd_vel_selected` and its output
`cmd_vel_smoothed` to the driver's `cmd_vel`, using relative topic names inside
the robot namespace. Begin with `OPEN_LOOP`; try `CLOSED_LOOP` only after
confirming that odometry twist is body-frame, high-rate, low-latency, and
correctly signed.

Measure and tune maximum comfortable acceleration, stopping distance, lateral
response, yaw response, tracking error, and Wi-Fi/ROS jitter. Tune x and y
independently because mecanum lateral performance may be weaker. These example
limits are test values, not hardware specifications.

## References

- [Nav2 Controller Server](https://docs.nav2.org/configuration/packages/configuring-controller-server.html)
- [Nav2 Velocity Smoother](https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html)
- [Nav2 behavior trees](https://docs.nav2.org/behavior_trees/)
