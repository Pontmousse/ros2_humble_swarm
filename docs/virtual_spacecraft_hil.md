# Virtual spacecraft HIL mode

The `swarm_spacecraft` package turns each RoboMaster into a **motion renderer**
for an ideal planar spacecraft. The software integrates a virtual force and yaw
torque into an ideal reference state; a feedback loop then commands the physical
Mecanum chassis to shadow that state. The floor friction is not removed—the
motors compensate for it while the virtual state retains momentum.

## Signal flow

```text
spacecraft controller                         physical localization
 geometry_msgs/Wrench                               nav_msgs/Odometry
          |                                                  |
          v                                                  v
 exact planar rigid-body integrator ---> reference + pose-error tracker
                                              |
                                              v
                                     geometry_msgs/Twist
                                              |
                                              v
                                      RoboMaster cmd_vel
```

All topics and services are relative, so launching in namespace `RM1` produces:

| Interface | Type | Purpose |
| --- | --- | --- |
| `RM1/spacecraft_wrench` | `geometry_msgs/msg/Wrench` | Virtual `Fx`, `Fy`, and `torque.z` input |
| `RM1/odom` | `nav_msgs/msg/Odometry` | Physical pose feedback |
| `RM1/cmd_vel` | `geometry_msgs/msg/Twist` | Body-frame chassis command |
| `RM1/virtual_spacecraft/odom` | `nav_msgs/msg/Odometry` | Ideal reference for logging/visualization |
| `RM1/virtual_spacecraft/reset` | `std_srvs/srv/Trigger` | Copy physical pose into a stationary virtual state |
| `RM1/virtual_spacecraft/enable` | `std_srvs/srv/SetBool` | Enable or pause physical motion |

Only planar wrench components are used. By default force is interpreted in the
virtual spacecraft body frame and rotated into the laboratory `map` frame.
Set `wrench_in_body_frame: false` for an inertial-frame controller.

## Dynamics and tracking

For a constant wrench during `dt`, the integrator uses exact
constant-acceleration kinematics:

```text
r[k+1] = r[k] + dt v[k] + dt^2 F[k] / (2m)
v[k+1] = v[k] + dt F[k] / m
yaw[k+1] = yaw[k] + dt rate[k] + dt^2 torque[k] / (2J)
rate[k+1] = rate[k] + dt torque[k] / J
```

The inertial tracking command is `v_ref + Kp (r_ref - r_measured)`. It is
rotated using **measured** yaw into the robot body frame. Linear saturation
preserves direction, while yaw rate is independently bounded.

This first implementation models force-free planar translation and yaw. An HCW
orbital propagator can be added behind the same tracker later; keeping propagation
and physical tracking separate is the important architectural boundary.

## Build and run

```bash
cd ros2_swarm
colcon build --symlink-install --packages-select \
  swarm_interfaces swarm_spacecraft swarm_bringup
source install/setup.bash

export ROBOT_IDX=1
ros2 launch swarm_bringup launch_virtual_spacecraft.py
```

The normal RoboMaster driver and localization pipeline must also be running, but
no other node may publish to the same robot's `cmd_vel` concurrently.

After valid odometry arrives, reset and explicitly arm the renderer:

```bash
ros2 service call /RM1/virtual_spacecraft/reset std_srvs/srv/Trigger '{}'
ros2 service call /RM1/virtual_spacecraft/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic pub --rate 20 /RM1/spacecraft_wrench geometry_msgs/msg/Wrench \
  '{force: {x: 1.0}, torque: {z: 0.0}}'
```

Stop the wrench publisher to remove virtual force. The virtual velocity then
remains constant, which is the desired free-drift behavior. Pause physical
motion with:

```bash
ros2 service call /RM1/virtual_spacecraft/enable std_srvs/srv/SetBool '{data: false}'
```

## Safety and experimental validation

- The node starts disabled and requires odometry before it can be enabled.
- Stale odometry produces a zero command; a stale wrench becomes zero rather
  than allowing an accidentally latched force to accelerate indefinitely.
- Long scheduler gaps are clamped with `maximum_time_step`.
- Input wrench and output velocities are bounded independently.
- Reset also disables the renderer so repositioning cannot cause sudden motion.
- Hardware emergency-stop behavior must remain independent of ROS software.

Before multi-robot testing, validate one robot in a clear test area. Record
physical and virtual odometry, command saturation, timing, and tracking error.
Identify chassis bandwidth and increase gains gradually. Persistent saturation
means the laboratory time/length scaling or virtual input limits are too
aggressive and invalidates faithful motion rendering.

## Nav2 side note

Nav2 is useful for map-based navigation, localization integration, collision
checking, lifecycle management, and sending a robot to a starting pose. It is
not a replacement for this renderer: its planners and regulated controllers are
designed to converge to paths and poses, whereas a free-floating spacecraft must
retain nonzero velocity when force is removed. Nav2 can therefore supervise
setup, geofencing, or recovery around the experiment, but the virtual propagator
and high-rate velocity tracker should remain a dedicated controller. Also ensure
that Nav2 and this node never command `cmd_vel` simultaneously; use an explicit
velocity multiplexer if both are installed.
