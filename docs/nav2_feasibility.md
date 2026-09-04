# Nav2 feasibility and architecture for the RoboMaster swarm

## Executive conclusion

Nav2 is feasible on the RoboMaster S1/EP platform in this repository, but it is
not a replacement for the swarm algorithm. The best separation of concerns is:

- the **swarm layer** decides *where each robot should go* and why (formation,
  encirclement, target assignment, capture sequencing, consensus);
- **Nav2 on each robot** decides *how that robot safely and smoothly reaches its
  assigned pose or follows its assigned path*;
- the RoboMaster driver remains the hardware adapter consuming `cmd_vel`.

Nav2 can bring path tracking, acceleration constraints, progress/goal checking,
recovery behavior, obstacle-aware planning and control, lifecycle supervision,
and standard navigation actions. It cannot by itself provide multi-robot task
allocation, formation stability, a good global pose estimate, or collision
sensing that the hardware does not supply.

The recommended path is incremental. First standardize localization and TF and
put command arbitration plus Nav2's velocity smoother in front of the driver.
Then let the existing swarm guidance emit goals/paths to a per-robot Nav2
controller. Add costmaps and a planner only after a usable obstacle sensor or
trusted external obstacle map is available. This captures most of the control
stability benefit without turning Nav2 into the swarm coordinator.

## What the repository does today

The centralized launch creates a complete pipeline inside each `RMx` namespace:

```text
Marvelmind position ----+
                        +--> pose_publisher --> pose (StatePos)
RoboMaster odom/yaw ----+

camera --> ArUco --> landmarks/targets --> guidance --> target_position/orientation
                                                    |
                                                    v
                                        proportional controller
                                                    |
                                                    v
                                                cmd_vel
                                                    |
                                                    v
                                           RoboMaster driver
```

More specifically:

- `pose_publisher` takes Marvelmind `mm_pos` for absolute `x/y`, takes yaw from
  RoboMaster `odom`, performs an initial offset calibration, and publishes the
  custom `StatePos` message. This is a direct composition, not covariance-aware
  fusion.
- `position_guidance3` computes landmark attraction and neighbor repulsion and
  publishes a new target point as `pose + velocity-like potential`. The point is
  refreshed every timer tick rather than represented as a time-stamped pose or
  path.
- `pointing_guidance2` points toward the landmark centroid and publishes a
  desired heading in degrees.
- `ep_proportional_controller` applies independent proportional position and
  heading control, a deadband and hard velocity clipping, transforms the global
  translational command to the body frame, and publishes `Twist`.
- `cp_proportional_controller` bypasses global navigation for close-range visual
  docking/capture. It drives from target markers in the camera frame and invokes
  the wheel/magnet services near the target.
- The RoboMaster driver already publishes standard `nav_msgs/Odometry`, IMU and
  `odom -> base_link` TF (namespaced frame IDs), accepts an omnidirectional
  `Twist`, and maps ROS `linear.x`, `linear.y`, and `angular.z` to the chassis.

This is a useful foundation: the base is already velocity controlled and
holonomic, all robots are namespaced, a URDF exists, and absolute position is
available. The largest integration gap is not the base driver; it is the
standard navigation state contract (`map -> odom -> base_link`, stamped poses,
covariances) and obstacle perception.

## Nav2 in ROS 2 terms

Nav2 is a collection of lifecycle-managed servers coordinated through actions,
normally by a behavior tree (BT). It is deliberately modular:

| Component | Responsibility | Relevance here |
|---|---|---|
| BT Navigator | Executes `NavigateToPose` / `NavigateThroughPoses`, including retries and recoveries | A standard action boundary between swarm missions and single-robot navigation |
| Planner server | Computes a global `nav_msgs/Path` through a global costmap | Useful for rooms/obstacles; optional for local potential-field motion |
| Controller server | Tracks a path and outputs velocity commands using a controller plugin | The main candidate to replace the present P controller |
| Smoother server | Smooths geometric paths, not velocity commands | Useful when swarm-generated paths are jagged |
| Velocity smoother | Enforces velocity, acceleration and deceleration limits and can use odometry feedback | Immediate improvement to command continuity and stopping behavior |
| Collision monitor | Filters or stops velocity using fresh sensor observations in configured safety zones | Valuable as an independent last safety layer, but requires suitable observations |
| Costmaps | Fuse static and live obstacle layers around the robot | Needed for obstacle-aware planning/control, not required merely to smooth commands |
| Behavior server | Supplies recovery actions such as wait, backup or spin | Useful only where those behaviors make sense for an omnidirectional swarm |
| Lifecycle manager | Configures, activates, monitors and can respawn a related set of Nav2 nodes | Cleaner startup/failure handling than fixed initialization delays |

A usual request flows as follows:

```text
mission client
  -> NavigateToPose action
  -> BT Navigator
       -> planner server: goal + map/costmap -> global Path
       -> controller server: Path + TF + local costmap -> cmd_vel_nav
       -> progress/goal checkers report success or failure
       -> BT selects retry/replan/recovery/abort
  -> velocity smoother
  -> collision monitor
  -> command mux
  -> RoboMaster cmd_vel
```

Nav2 does **not** close the motor loop. Its controller plugins calculate chassis
velocity setpoints; the RoboMaster firmware/driver executes those setpoints.
Controller stability therefore depends on localization latency/noise, controller
frequency and tuning, feasible acceleration limits, driver timeout behavior,
network jitter, and the robot's onboard response.

Useful upstream references for ROS 2 Humble are the [Nav2 concepts overview](https://docs.nav2.org/concepts/index.html),
[navigation plugins](https://docs.nav2.org/plugins/index.html),
[configuration guide](https://docs.nav2.org/configuration/index.html), and
[first-time robot setup guide](https://docs.nav2.org/setup_guides/index.html).
Use the Humble packages actually installed in the target image as the source of
truth for available plugins and parameter names; newer Nav2 documentation can
describe features absent from Humble.

## What Nav2 could improve

### 1. More stable low-level guidance

The current controller has position/heading P gains, deadbands, saturation, and
no explicit acceleration or jerk policy. Targets can change at 100 Hz. That can
produce discontinuous saturated commands, oscillation around a moving target,
and abrupt stops. A velocity smoother can provide asymmetric acceleration and
deceleration limits, command timeouts, deadband handling, and optionally
closed-loop feedback from odometry. This is the lowest-risk Nav2 adoption and
can be used even while retaining the current controller.

The smoother should output to an intermediate topic such as
`/<robot>/cmd_vel_smooth`, not directly share `cmd_vel` with other publishers.
A final mux/safety owner must be the **only** publisher to the driver's
`/<robot>/cmd_vel`. Set the driver's currently-disabled `chassis.timeout` to a
non-zero value so stale commands stop the base even if upstream nodes die.

### 2. Better path tracking than point chasing

Instead of publishing an instantaneous `CoordXY`, swarm guidance can publish a
short, stamped `nav_msgs/Path` in the global swarm frame. A Nav2 controller
plugin then tracks the path with velocity and acceleration constraints. This
separates the slower swarm policy from the faster local feedback loop and gives
well-defined progress, goal tolerances, cancellation, and failure reporting.

The RoboMaster is mecanum/holonomic, so the chosen controller must explicitly
support lateral `linear.y`, and its kinematic/velocity parameters must be tuned
accordingly. DWB is a conservative first candidate because it supports
holonomic velocity sampling. A controller that assumes differential drive may
still navigate by turning and driving forward, but wastes the platform's
lateral authority and may conflict with the desired facing direction. Evaluate
the precise Humble version of DWB and any alternative plugin in simulation and
on one robot before committing to it.

### 3. Standard action and supervision semantics

Replacing ad-hoc target topics and toggle services at the navigation boundary
with actions provides goal acceptance, feedback, cancellation, results and
timeouts. The swarm coordinator can assign a goal to each robot, monitor all
action clients, cancel or reassign only the affected member, and implement
group-level policy without directly publishing wheel commands. Nav2 lifecycle
nodes also make readiness explicit; a coordinator can wait for the navigation
stack to become active instead of relying on `2 * init_period` sleeps.

### 4. Obstacle-aware motion and recoveries

With a static occupancy map and/or adequate range data, Nav2 can plan around
walls and track paths against rolling local costmaps. Its critics can score
obstacle clearance rather than relying only on landmark and neighbor potential
fields. Collision Monitor can enforce stop/slow zones independently of the
planner.

This benefit is currently conditional. The camera pipeline detects ArUco
markers, not general obstacles, and the optional RoboMaster ToF devices are
individual ranging sensors rather than an existing `LaserScan`/`PointCloud2`
obstacle source in this workspace. Nav2 cannot avoid unobserved people, robots,
or furniture. A practical sensor path is one of:

1. add a 2-D lidar;
2. add/calibrate a depth camera and publish a filtered point cloud;
3. convert multiple correctly mounted ToF beams to a standard, time-stamped
   observation source, accepting sparse coverage;
4. publish obstacles from an external tracking system into each robot's local
   costmap, with conservative age/latency checks.

### 5. Diagnostics and repeatable tuning

Standard paths, costmaps, controller evaluation messages, actions, lifecycle
state, TF and odometry are easier to inspect in RViz, bags, PlotJuggler and
automated tests than custom degree-based point messages. This will likely make
the system *operationally* more stable even where the control law is similar,
because stale localization, blocked progress, and failed planning become
observable states instead of silent behavior.

## What Nav2 will not solve

- **Swarm coordination:** it has no built-in formation, encirclement, neighbor
  consensus, target allocation, capture synchronization or multi-robot traffic
  reservation.
- **Localization quality:** Nav2 consumes TF; it does not fuse Marvelmind and
  odometry. Bad jumps, timestamps, frame conventions or yaw offsets remain bad
  inputs.
- **Inter-robot collision guarantees:** representing peers as dynamic obstacles
  helps reactive avoidance but does not prevent deadlocks or give space-time
  guarantees. Narrow-passage coordination belongs in a fleet/swarm layer.
- **Docking/capture:** close-range ArUco alignment and magnet sequencing are a
  task-specific behavior. Keep the current capture controller initially, behind
  command arbitration, or later implement it as a custom BT/action server.
- **Hard real-time control:** Nav2 is a ROS-level navigation framework over Wi-Fi
  and the Python RoboMaster SDK, not a deterministic motor controller.

## Architecture choices

### Option 0 — Improve the current stack without Nav2

Keep the guidance and P controller, but add standard messages/TF, filtered state
estimation, a command mux, acceleration limiting, watchdogs, timestamps and
automated tests.

**Pros:** smallest footprint and complete control of the algorithm.
**Cons:** reimplements navigation lifecycle, action, progress and recovery
machinery; remains point chasing.
**Use when:** the environment is open and obstacle-free and research focuses on
the swarm law rather than autonomous navigation.

### Option 1 — Nav2 utilities around the existing controller (recommended first)

```text
existing guidance -> existing P controller -> cmd_vel_raw
                                         -> velocity_smoother
                                         -> collision_monitor (when sensor exists)
teleop ------------------------------------> cmd_vel_mux -> driver/cmd_vel
capture controller ------------------------> /
```

**Pros:** quickest stability and safety improvement; preserves current behavior;
allows measured A/B comparison.
**Cons:** no path planning or Nav2 progress/recovery semantics.
**Effort:** low, after topic ownership and driver timeout are corrected.

### Option 2 — Swarm-generated paths, per-robot Nav2 control (recommended target)

```text
                  GLOBAL / SWARM PLANE
Marvelmind + peer states -> swarm coordinator/policy
                              | one PoseStamped or Path per robot
            +-----------------+------------------+
            v                                    v
     RM1 namespace                         RM2 namespace
state estimator                            state estimator
map->odom->base_link                       map->odom->base_link
Nav2 controller server                     Nav2 controller server
velocity smoother                          velocity smoother
collision monitor/mux                      collision monitor/mux
RoboMaster driver                          RoboMaster driver
```

The swarm policy remains authoritative over formation/neighbor constraints. It
can produce a receding-horizon path for each robot, while each controller server
tracks locally. A thin adapter may use `FollowPath`, or a custom Nav2 global
planner plugin can expose the swarm path through the usual Navigate action.

**Pros:** clean research boundary, better tracking and introspection, supports
holonomic motion, planner/costmaps can be added later.
**Cons:** rapidly replacing a path can cause controller churn; peer constraints
must remain coherent; controller behavior without a meaningful local costmap
needs validation.
**Effort:** medium.

### Option 3 — Full Nav2 stack per robot

Each robot runs localization, global/local costmaps, planner, controller,
behavior server, BT Navigator, smoothers and lifecycle manager. The swarm
coordinator sends `NavigateToPose` or `NavigateThroughPoses` actions.

**Pros:** standard autonomous navigation, obstacle-aware global planning,
recoveries and mature tooling.
**Cons:** highest CPU/network/configuration cost; independent planners can
deadlock or fight formation objectives; needs an obstacle perception system and
consistent map.
**Effort:** high. Use for point-to-point missions in mapped spaces, not as the
first experiment.

### Option 4 — Central multi-robot planner plus local Nav2 execution

A central component generates mutually compatible paths (ideally trajectories
with time/reservations), while per-robot Nav2 controllers execute them.

**Pros:** can address robot-robot conflicts and narrow passages deliberately.
**Cons:** central failure/communications dependency; Nav2 paths are geometric,
not time-parameterized multi-agent trajectories, so an execution/scheduling
contract is still required.
**Effort:** very high. Consider only if collision-free coordinated routing is a
primary requirement.

## Recommended target design

### State estimation and frames

For every robot, establish this standard TF tree:

```text
swarm_map  --dynamic correction-->  RMx/odom  --continuous local odom-->  RMx/base_link
                                                                      \-> sensors
```

- `swarm_map` is the common Marvelmind coordinate frame. Its axes, origin and
  yaw convention must be calibrated once and documented.
- The RoboMaster driver continues to own `RMx/odom -> RMx/base_link` because it
  is locally smooth.
- A state estimator owns `swarm_map -> RMx/odom`, combining absolute Marvelmind
  position with wheel/chassis odometry and IMU. `robot_localization` is a strong
  first choice; a carefully tested custom estimator is also possible.
- Never let two nodes publish the same TF edge. Keep all angles in radians at
  standard ROS boundaries. Publish covariances and original measurement stamps;
  do not stamp delayed measurements as “now.”
- Continue ROS namespaces for topics/actions, but use explicit unique frame IDs
  because TF is not automatically isolated by namespaces.

The current `pose_publisher` can remain temporarily as a compatibility adapter
from TF/odometry back to `StatePos`, but controllers should migrate to TF and
standard stamped messages. The experimental custom Kalman filter should not be
used as the Nav2 localization source without correction and validation: its
odometry measurement currently assigns `linear.x` to both `vx` and `vy`, and its
state/measurement model and covariances are fixed in code.

### Goal ownership and command ownership

Define two explicit arbitration points:

1. **Goal owner:** the swarm coordinator owns long-range assignments. It sends
   action goals/paths; it never publishes final chassis commands.
2. **Velocity owner:** exactly one mux/safety chain publishes the driver's
   `cmd_vel`. Inputs have priority and timeout, for example:
   `emergency stop > manual teleop > capture/docking > Nav2 > legacy controller`.

On mode changes, cancel the old action/controller, wait for acknowledgement,
send zero velocity, then enable the new source. Topic multiplexing alone does
not cancel a Nav2 action, so both goal state and velocity state must be switched.

### Neighbor handling

There are three non-exclusive strategies:

- keep neighbor repulsion in the swarm path generator (recommended initially);
- inject peer footprints into costmaps as dynamic obstacles for a local safety
  backstop, with unique IDs and short observation persistence;
- use centralized reservation/trajectory coordination where guarantees matter.

Do not feed a robot its own footprint as an obstacle. Account for Marvelmind and
network latency by predicting peer pose or inflating peer footprints. Avoid
double-counting an aggressive repulsion term in both the swarm policy and local
costmap tuning, which can cause oscillation.

### Capture behavior

Retain `cp_proportional_controller` as a separate docking action first:

1. Nav2 reaches a staging pose near the visual target.
2. The BT/coordinator cancels Nav2 and confirms zero velocity.
3. The docking action exclusively acquires the mux, performs ArUco alignment,
   engages the magnet/wheels, and returns success/failure.
4. On target loss or timeout it commands zero and releases control.

Later, this action can be called from a custom Nav2 behavior-tree node without
putting camera-space servoing inside a general navigation controller.

## Migration plan and decision gates

### Phase 0 — Measure the baseline

Bag, for one robot, Marvelmind, odometry, IMU, current pose/targets, `cmd_vel`,
TF and capture events. Measure step response, steady-state error, overshoot,
settling time, command discontinuity, pose age/dropout, and stop distance. This
is necessary to know whether Nav2 actually improves stability.

**Gate:** repeatable baseline and written coordinate/sign/unit checks.

### Phase 1 — Standard navigation contract

1. Calibrate `swarm_map` against RoboMaster odometry axes and yaw.
2. Publish stamped standard observations with realistic covariances.
3. Configure an estimator for the standard TF chain.
4. Add TF freshness/jump tests and RViz visualization.
5. Enable a non-zero driver command timeout.

**Gate:** continuous local odometry, bounded global correction, no duplicate TF
publishers, and reliable transforms at the intended controller rate.

### Phase 2 — Safe velocity pipeline

Add a mux and velocity smoother under a new output topic, initially with no
Nav2 planner. Start with conservative lateral/longitudinal/angular acceleration
and deceleration measured from hardware. Test loss of controller, estimator,
Marvelmind and Wi-Fi. Add Collision Monitor only with a validated observation
source.

**Gate:** every stale/crashed input stops within the stated distance and time;
manual override always wins.

### Phase 3 — One-robot controller proof of concept

Run a namespaced Nav2 controller on one robot. Convert a fixed waypoint sequence
to a `Path`, tune footprint and holonomic limits, and compare DWB tracking with
the legacy P controller on identical bags/courses. Test forward, lateral,
diagonal, rotate-in-place and simultaneous translation/rotation.

**Gate:** equal or better tracking/settling, no command sign errors, acceptable
CPU/load and robust cancel/preemption.

### Phase 4 — Swarm-generated paths

Adapt guidance to produce bounded-horizon stamped paths or action goals at a
lower policy rate. Run Nav2 control at a stable higher rate. Preserve the legacy
controller behind a launch option for A/B and fallback. Add peer footprints to
costmaps only after baseline swarm behavior is stable.

**Gate:** formation/capture metrics do not regress as robot count and network
load increase; no self-obstacle or namespace leakage.

### Phase 5 — Planning and richer safety, if required

Add a static map and validated obstacle source, then global/local costmaps,
planner and selected recoveries. Tune in simulation before hardware. Only then
consider a full BT mission or central conflict-aware planner.

## Initial configuration checklist

- Install the Humble Nav2 packages in the runtime image; they are not currently
  listed explicitly in `Dockerfile.src`.
- Give every Nav2 node, action, topic and frame a robot namespace/prefix.
- Use a footprint close to the real RoboMaster envelope plus attachments, not a
  zero-radius point or unverified generic radius.
- Set nonzero `max_vel_y`/sampling for a holonomic controller and validate all
  signs against the driver's `y` and yaw conversions.
- Separate swarm-policy frequency, Nav2 controller frequency, velocity-smoother
  frequency and hardware odometry rate; 100 Hz Python timers do not guarantee
  100 Hz fresh measurements or commands.
- Configure progress/goal checkers for moving/receding targets deliberately.
  Do not expect ordinary `NavigateToPose` completion if the goal is replaced at
  every guidance tick.
- Prefer sensor-data QoS for high-rate observations and reliable actions for
  goals; choose QoS per data semantics rather than one profile for every topic.
- Record the exact Humble package versions and configuration with each trial.
- Test lifecycle shutdown, action cancellation, localization dropout, stale
  obstacle data, driver disconnect, Wi-Fi loss and emergency-stop behavior.

## Practical recommendation

Proceed with Nav2, but scope the first deliverable as **state/TF standardization
+ command mux + velocity smoother + one-robot FollowPath experiment**. Do not
start by launching four complete navigation stacks or replacing capture
servoing. If the one-robot experiment demonstrates better tracking and recovery,
adopt Option 2: keep swarm intelligence above Nav2 and run a controller/safety
pipeline per robot. Move to full planner/costmap Nav2 only when obstacle-aware
navigation is an actual requirement and an observation source exists.

This design keeps the novel swarm work in this repository, uses Nav2 for the
well-understood navigation machinery it is good at, and gives clear rollback
points throughout the migration.
