# Virtual spacecraft HIL mode

The `swarm_spacecraft` and `swarm_nav2_controller` packages turn each RoboMaster
into a **motion renderer** for an ideal planar spacecraft. The simulator
integrates a virtual force and yaw torque into an ideal reference state. A
selectable P+FF or DWB controller commands the physical Mecanum chassis to
shadow that state. The floor friction is not removed—the motors compensate for
it while the virtual state retains momentum.

## Signal flow

```text
spacecraft controller
 geometry_msgs/Wrench
          |
          v
 virtual_spacecraft --------> virtual_spacecraft/odom
                                      |
                         +------------+------------+
                         |                         |
                         v                         v
localization/odom ---> nav2_pff              nav2_dwb_path
                         |                         |
                         |                   FollowPath
                         |                         v
                         |               controller_server + DWB
                         |                         |
                         +-------> cmd_vel_raw <---+
                                      |
                                      v
                              velocity_smoother
                                      |
                                      v
                                   cmd_vel
```

All topics and services are relative, so launching in namespace `RM1` produces:

| Interface | Type | Purpose |
| --- | --- | --- |
| `RM1/spacecraft_wrench` | `geometry_msgs/msg/Wrench` | Virtual `Fx`, `Fy`, and `torque.z` input |
| `RM1/odom` | `nav_msgs/msg/Odometry` | Raw RoboMaster attitude and body twist |
| `RM1/mm_pos` | `marvelmind_ros2_msgs/msg/HedgePositionAddressed` | Filtered physical x/y position |
| `RM1/localization/odom` | `nav_msgs/msg/Odometry` | Physical GPS x/y plus calibrated RoboMaster yaw and twist |
| `RM1/virtual_spacecraft/odom` | `nav_msgs/msg/Odometry` | Ideal reference for logging/visualization |
| `RM1/virtual_spacecraft/reset` | `std_srvs/srv/Trigger` | Copy physical pose into a stationary virtual state |
| `RM1/cmd_vel_raw` | `geometry_msgs/msg/Twist` | Selected controller output before smoothing |
| `RM1/cmd_vel` | `geometry_msgs/msg/Twist` | Smoothed body-frame command sent to the driver |
| `RM1/follow_path` | `nav2_msgs/action/FollowPath` | One-pose moving-goal input for DWB modes |
| `RM1/bounding_box_search/markers` | `visualization_msgs/msg/MarkerArray` | Optional bounded-search experiment visualization |

Only planar wrench components are used. By default force is interpreted in the
virtual spacecraft body frame and rotated into the laboratory `swarm_map` frame.
Set `wrench_in_body_frame: false` for an inertial-frame controller.

The existing `pose_publisher` also publishes `localization/odom`. Its pose is
expressed in `swarm_map`: x/y come directly from the filtered `mm_pos` topic,
while yaw and body-frame twist come from RoboMaster odometry. This is direct
message composition, not another state estimator. The legacy `pose` topic is
unchanged. Its child frame preserves the driver's namespaced base frame, while
virtual odometry uses a namespace-unique frame such as
`RM1/virtual_spacecraft`.

For Nav2, `pose_publisher` also computes the correction transform
`swarm_map -> RM1/odom` from the calibrated global pose and the matching raw
driver pose. The RoboMaster driver remains the only publisher of
`RM1/odom -> RM1/base_link`.

For consistency with the other swarm nodes, `timer_frequency` is the timer
period in seconds; the default `0.01` therefore means 100 Hz. Both HIL nodes
also expose `qos_depth`, `qos_reliability`, and `qos_history`, with reliable,
keep-last defaults.

## Dynamics and tracking

For a constant wrench during `dt`, the integrator uses exact
constant-acceleration kinematics:

```text
r[k+1] = r[k] + dt v[k] + dt^2 F[k] / (2m)
v[k+1] = v[k] + dt F[k] / m
yaw[k+1] = yaw[k] + dt rate[k] + dt^2 torque[k] / (2J)
rate[k+1] = rate[k] + dt torque[k] / J
```

The separate `nav2_pff` node converts the reference body velocity to the global
frame and applies `v_ref + Kp (r_ref - r_measured)`. It rotates that result using
**measured** yaw into the robot body frame. Linear saturation preserves
direction, while yaw rate is independently bounded.

This first implementation models force-free planar translation and yaw. An HCW
orbital propagator can be added behind the same tracker later; keeping propagation
and physical tracking separate is the important architectural boundary.

## Build and run

```bash
cd ros2_swarm
colcon build --symlink-install --packages-select \
  swarm_interfaces swarm_filter swarm_spacecraft swarm_controller \
  swarm_nav2_controller \
  swarm_dwb_critics \
  swarm_bringup
source install/setup.bash

export ROBOT_IDX=1
ros2 launch swarm_bringup launch_virtual_spacecraft.py
```

The normal RoboMaster driver, Marvelmind pipeline, position filter, and
`pose_publisher` must also be running. No other node may publish to the same
robot's `cmd_vel` concurrently.

The launch file starts the simulator, selected tracker, and the standard Nav2
velocity smoother. P+FF remains the default:

```bash
ros2 launch swarm_bringup launch_virtual_spacecraft.py controller_mode:=pff
```

Select one of the moving-goal DWB experiments with:

```bash
ros2 launch swarm_bringup launch_virtual_spacecraft.py controller_mode:=dwb
ros2 launch swarm_bringup launch_virtual_spacecraft.py controller_mode:=dwb_ff
ros2 launch swarm_bringup launch_virtual_spacecraft.py controller_mode:=dwb_velocity_feedback
```

Enable the optional no-target bounded-search wrench guidance and RViz markers
with any controller mode:

```bash
ros2 launch swarm_bringup launch_virtual_spacecraft.py \
  controller_mode:=pff guidance_mode:=bounding_box \
  enable_guidance_visualization:=true
```

Its behavior and visualization contract are documented in
[`bounding_box_search_hil.md`](bounding_box_search_hil.md).

Each DWB mode sends only the current virtual pose as a one-pose path. This is
intentionally a moving-goal experiment, not prediction of the future virtual
trajectory. The feedforward mode adds a critic that prefers the virtual body
velocity. The feedback mode additionally corrects that target with measured
physical velocity error using initial gain `0.3`.

Tracking begins automatically after physical and virtual odometry are
available. Reset the virtual state and publish a wrench with:

```bash
ros2 service call /RM1/virtual_spacecraft/reset std_srvs/srv/Trigger '{}'
ros2 topic pub --rate 20 /RM1/spacecraft_wrench geometry_msgs/msg/Wrench \
  '{force: {x: 1.0}, torque: {z: 0.0}}'
```

Stop the wrench publisher to remove virtual force. The virtual velocity then
remains constant, which is the desired free-drift behavior. Stop the tracker or
the combined launch to stop motion rendering; hardware emergency-stop behavior
must remain independent of ROS software.

## Capture preparation

During ENCAPSULATION, `cp_controller` publishes `preparation_wrench` for a fresh,
locked assigned attachment point present in fresh estimated `targets`. It uses
the same tangential bearing-alignment command and filtered bearing-rate damping
as capture. Estimated geometry allows this preparation to recover raw visibility.

The body-frame force projects the velocity-servo error onto the AP tangent:
`F = clamp(Kv * (alignment_speed - dot(craft_velocity, tangent)), ±preparation_max_force) * tangent`.
It adds no radial attraction, radial braking, target-velocity feedforward, or
torque. `preparation_max_force` defaults to **0.002 N** (2 mN) in both swarm
launches; set it to zero to disable the source for comparison. This is a starting
cap for the current 20 mN total propulsion limit, not a validated tuning result.

Preparation publishes zero in SEARCH, CAPTURE, DOCK, on assignment/lock/target
loss or timeout, and while either capture or dock service flag is active. Missing
assigned geometry never falls back to another point for preparation. The existing
capture source keeps its closest-point fallback, radial approach, feedforward,
and heading law. The shared alignment term is `K * bearing + D * bearing_rate`:
decreasing bearing reduces the requested sidling speed. `pointing_guidance2`
publishes zero torque in CAPTURE and DOCK, leaving capture heading and yaw-rate
damping to CP; centroid pointing remains active in SEARCH/ENCAPSULATION.
AP identity changes and long
gaps reset the shared bearing-rate history. The source is also published in
`DIRECT_CMD_VEL` mode, where the existing nav passthrough tracks the virtual
spacecraft before capture; no additional `cmd_vel` writer is introduced.

Readiness now allows **2.0 seconds** since the last live raw observation of the
assigned AP (`perception_stale_timeout`). The raw-observation gate remains active.
This is observation freshness, not a sensing period or the synchronization
tolerance; `action_lead` remains 2 seconds. An observation made at proposal time
alone will expire around execution time, so subsequent observations are needed.

The bag recorder includes `preparation_wrench`, assigned AP, assignment lock,
current mode, capture clearance, consensus debug, and global `/readiness` and
`/ap_bids`, for whichever robots the recorder selects. Select both robots when
investigating a two-agent run.

## RViz capture timing

With `enable_guidance_visualization:=true`, the first selected robot's visualizer
adds one shared table to its existing `spacecraft_visualizer/markers` topic. Keep
that MarkerArray display enabled in RViz. The table appears inside the top of the bounding box
in `swarm_map`, even before odometry arrives; no extra plugin is needed.

The header shows current **ROS time**, and each row shows a readiness `robot_id`,
its proposed absolute action time, seconds remaining, and WAIT/READY/ARMED status.
All times use seconds to three decimal places. A negative countdown means the
deadline has passed; ARMED does not confirm capture has fired. Missing proposals
show `--`; rows become STALE after one second without updates. ROS time and the
countdown pause together when using a paused simulation clock. The visualizer and
consensus nodes must use the same `use_sim_time` setting.

Position and size can be configured under `/**/spacecraft_visualizer` in
`virtual_spacecraft.yaml` with `timing_table_x`, `timing_table_y`, and
`timing_table_text_size`. Defaults are the box's X midpoint, `y_max - 0.15` for the
table's top edge, and 0.14 m text height. Rows extend downwards from that edge.
Standalone visualizers default to hiding the table; enable
`show_capture_timing:=true` on one instance. This is a read-only overlay.

## Safety and experimental validation

- `localization/odom` is withheld until GPS and RoboMaster attitude initialize
  and while either underlying input is stale.
- The tracker publishes zero until physical and virtual odometry are available
  in the same non-empty global frame.
- Stale physical or virtual odometry produces a zero command; a stale wrench
  becomes zero rather than allowing an accidentally latched force to accelerate
  indefinitely.
- A stale DWB reference cancels the active `FollowPath` goal, and the smoother
  times out to zero if raw controller commands stop.
- Long scheduler gaps are clamped with `maximum_time_step`.
- Input wrench and output velocities are bounded independently.
- Hardware emergency-stop behavior must remain independent of ROS software.

Before multi-robot testing, validate one robot in a clear test area. Record
physical and virtual odometry, command saturation, timing, and tracking error.
Identify chassis bandwidth and increase gains gradually. Persistent saturation
means the laboratory time/length scaling or virtual input limits are too
aggressive and invalidates faithful motion rendering.

## DWB experiment notes

The first DWB configuration uses a free rolling costmap without obstacle sensor
inputs and is intended only for a clear test area. All DWB configurations are
holonomic. The velocity-aware configurations share one C++ critic; feedforward
uses zero velocity-feedback gain, while the feedback experiment uses gain
`0.3`. These values are conservative starting points and must be tuned from
recorded chassis response.

Only the controller selected by `controller_mode` is launched. Do not start a
second controller that publishes to the same robot's `cmd_vel_raw` or bypass
the hardware emergency stop.
