# Bounding-box search HIL demonstration

## Purpose

Demonstrate the virtual spacecraft's free-floating motion and intermittent
thruster firings using a simple, visually understandable search inside a
rectangular bounding box.

This experiment is only the no-target search behavior. It does not include
landmarks, target detection, flocking, encapsulation, or capture.

## Experiment behavior

- The virtual spacecraft moves inside a rectangular bounding box.
- An initial short force firing injects translational velocity.
- The spacecraft coasts with exactly zero wrench between firings.
- Near a boundary, a finite inward force firing decelerates and reverses the
  outward velocity component.
- The bounce must result from applied force over time. The simulator must not
  directly reflect, replace, or reset the velocity state.
- An optional initial torque firing can inject angular velocity before
  zero-torque rotation.
- Guidance uses only `virtual_spacecraft/odom`. Physical localization must not
  close the outer guidance loop.
- The existing physical tracking controller remains the inner loop that makes
  the RoboMaster follow the virtual spacecraft.

## Visualization

- Draw the search region as a rectangle in the `swarm_map` plane.
- Draw the virtual spacecraft as a small triangle whose direction shows yaw.
- Draw the measured mobile robot as an optional faint second triangle.
- Draw applied force as a red straight arrow from the virtual spacecraft.
- Make force-arrow length proportional to force magnitude.
- Use a configurable visual scale so maximum force is approximately one or two
  plot units.
- A zero-force command has a zero-length arrow.
- Draw applied torque as an optional curved arrow whose direction shows the
  torque sign and whose size represents its magnitude.
- Recorded data should support replaying the same animation after the test.

## Implementation files

- `swarm_controller/swarm_controller/bounding_box_search.py`
  - Implements the initial firing, coasting, and boundary-firing state machine.
  - Subscribes to relative `virtual_spacecraft/odom`.
  - Publishes relative `spacecraft_wrench`.
  - Publishes zero wrench if virtual odometry is missing or stale.

- `swarm_controller/swarm_controller/bounding_box_visualizer.py`
  - Subscribes to virtual odometry, physical localization, and commanded
    wrench.
  - Publishes the rectangle, triangles, force arrow, and torque arrow as
    `visualization_msgs/msg/MarkerArray` for live viewing and rosbag replay.
  - Does not participate in either control loop.

- `swarm_bringup/config/virtual_spacecraft.yaml` (existing)
  - Stores bounding-box limits, boundary margin, firing magnitudes and
    duration, reference timeout, and visualization scales alongside the other
    HIL parameters.

- `swarm_bringup/launch/launch_virtual_spacecraft.py` (existing)
  - Adds the bounding-box guidance and visualizer nodes to the existing HIL
    launch.
  - Selects exactly one wrench guidance through `guidance_mode`; currently
    supported values are `none` and `bounding_box`.
  - Controls the visualizer independently through
    `enable_guidance_visualization`.
  - Keeps the experiment disabled by default so the existing launch behavior
    remains safe and backward compatible.
  - When selected, starts the guidance publisher alongside the virtual
    simulator, selected tracker, and velocity smoother.
  - Does not launch the legacy physical command-producing controllers. They
    must also be disabled in the separately launched hardware/localization
    pipeline.

## Topic and frame contracts

- `virtual_spacecraft/odom` (`nav_msgs/msg/Odometry`) is the only guidance
  feedback input. Its pose is in `swarm_map` and its twist is in the virtual
  spacecraft body frame.
- `spacecraft_wrench` (`geometry_msgs/msg/Wrench`) is the guidance output and
  virtual-dynamics input. Guidance calculates boundary forces in `swarm_map`
  and publishes their body-frame equivalent, preserving the simulator's
  existing `wrench_in_body_frame: true` convention. Torque uses `torque.z`.
- `localization/odom` (`nav_msgs/msg/Odometry`) is used only for the optional
  measured-robot overlay and by the physical tracking controller.
- `bounding_box_search/markers` (`visualization_msgs/msg/MarkerArray`) is
  visualization-only and uses `swarm_map`.
- `cmd_vel_raw` and `cmd_vel` retain the existing selected-controller and
  velocity-smoother contract.

All experiment topics are relative and every node is launched with an explicit
robot namespace. Robot selection remains unchanged for now. Extracting the
duplicated centralized, decentralized, and HIL robot-selection logic into a
shared helper is a separate future cleanup.

## Launch

The normal HIL launch remains unchanged when the new argument is omitted. Run
the bounded-search demonstration with, for example:

```bash
ros2 launch swarm_bringup launch_virtual_spacecraft.py \
  controller_mode:=pff guidance_mode:=bounding_box \
  enable_guidance_visualization:=true
```

Only one bounding-box guidance publisher should run in each robot namespace.
The configured box coordinates must be adjusted to the safe laboratory region
before operating hardware.

`x_min`, `x_max`, `y_min` and `y_max` are defined once in the top-level `/**`
block of `virtual_spacecraft.yaml` and are shared by `bounding_box_search` and
`bounding_box_visualizer`. Edit them in that one place; the enforced region and
the region drawn in RViz then cannot disagree. `boundary_margin` stays under
`bounding_box_search` because only the guidance node uses it.

### The box is a guidance target, not a hard limit

Rebound firings are bounded by `maximum_force`, and the boundary force is only
applied once the spacecraft is already inside `boundary_margin` of a wall. A
spacecraft arriving with enough momentum therefore crosses the wall and travels
some distance beyond it before the firing reverses its velocity. **Overshoot
during a bounce is expected behavior, not a fault.**

Overshoot distance grows with approach speed and shrinks with `boundary_margin`
and `boundary_force`. Size the physical safe area with clearance beyond the
configured box rather than treating the box edge as the limit of motion, and
expect the RViz rectangle to be crossed on every bounce.

In RViz, set the fixed frame to `swarm_map` and add a `MarkerArray` display for
the robot's `bounding_box_search/markers` topic.

## Data to record

- `virtual_spacecraft/odom`
- `localization/odom`
- `spacecraft_wrench`
- `cmd_vel_raw`
- `cmd_vel`
- `bounding_box_search/markers`

The main visible result should be: short firing, sustained zero-thrust drift,
boundary braking/reversal, and continued drift in the new direction.
