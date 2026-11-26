# Driver contract v1.0.0

The interface between our **control container** (this repo, `DRIVER_STACK=v2`) and the
**drivers container** (swarm_rover_v2 image, maintained upstream). Anything listed here is what
our side relies on; a change to any of it is a contract change and gets a new version file
(`driver-contract-vXYZ.md`), never an edit of this one.

| | |
|---|---|
| Contract version | **1.0.0** (2026-11-26) |
| Drivers image | `hostage03/robomaster_swarm_v2:app-4be73e1-drivers` (arm64 + amd64, public) |
| Upstream commit | `4be73e1` (jasonli03/swarm_rover_v2, branch env-docker). `0a30d2e` has the same `src/` and `config/` |
| Our branch | `split-containers` |
| Pinned in | `swarm_ansible_deploy/vars.yml` → `drivers_image` |

## Versioning rules

- **Major**: a breaking change. A `.msg` file changes; a topic, service or frame we use is
  renamed, retyped, removed or changes units, axes or stamp meaning; a container setting we depend
  on changes. Our side needs code changes and a new image.
- **Minor**: an additive change our side doesn't have to use. A new topic, service or param; a
  new node we may opt into via `drivers_nodes`.
- **Patch**: same interface, new image. Bug fixes, performance, defaults we override anyway.

**Every image bump** gets a new contract file, even for a patch, and a line in the
changelog at the bottom. Re-check the items in [Upgrade checklist](#upgrade-checklist).

## 1. Topics and services, per robot namespace `/RMx`

### Consumed by our side

| Topic | Type | Producer | QoS (pub) | Frame / units | Stamp | Rate | Our consumer |
|---|---|---|---|---|---|---|---|
| `odom` | `nav_msgs/Odometry` | chassis | reliable, depth 1 | `RMx/odom` → child `RMx/base_link`; x forward, y left, yaw CCW; twist in body frame, m/s, rad/s | `now()` at decode | 50 Hz (`odom_rate`) | pose_kf / pose_publisher, guidance |
| `gnss/position` | `geometry_msgs/PointStamped` | gnss | reliable, depth 1 | `marvelmind` (shared map frame, no robot prefix); metres | receive time − `latency_ms` (0) | per valid fix, ~6 Hz; **only published while subscribed** | `gnss_bridge` |
| `aruco/landmarks` | `swarm_interfaces/ArucoMarkerArray` | aruco_camera_2in1 | reliable, depth 10 | `RMx/camera_optical` (x right, y down, z forward); metres | V4L2 capture time | one per processed frame, capped by `max_rate` (15 Hz in our overlay); **empty arrays included** | `aruco_bridge` |
| `aruco/markers` | `swarm_interfaces/ArucoMarkerArray` | aruco_camera_2in1 | same | same | same header as `aruco/landmarks` | same | `aruco_bridge` |

Optional, not relied on: `imu` (`sensor_msgs/Imu`, off: `imu_rate: 0`), `battery`
(`sensor_msgs/BatteryState`), `diagnostics` and `health` (`DiagnosticArray`),
`camera/image_raw` and `camera/camera_info` (only while subscribed, for the dev PC).

### Provided by our side

| Name | Type | Our producer | Drivers behaviour |
|---|---|---|---|
| `cmd_vel` (topic) | `geometry_msgs/Twist` | velocity_smoother (or cp_controller with `DIRECT_CMD_VEL`) | body frame, m/s and rad/s; clamped to `max_vx`/`max_vy` 1.0 m/s, `max_wz` 3.14 rad/s; wheels stop after `cmd_timeout` 0.5 s without a message |
| `engage_wheels` (service, called by us) | `std_srvs/SetBool` | mode_supervisor on DOCK entry/exit | `true` engage, `false` free-wheel; waits for the robot's ack, **up to 3 s** |

## 2. Messages

Copied byte-identical into `ros2_swarm/src/swarm_interfaces/msg/`. Humble matches DDS types by
name and layout, so both sides must build the same definition. Check with `cmp` on every upgrade.

`ArucoMarker.msg`
```
# One detected ArUco marker, in the camera optical frame (x right, y down, z forward).
# Pose fields are NaN when the camera has no calibration.
int32 id
geometry_msgs/Pose pose          # position (m) + orientation (quaternion)
float64 distance                 # |position|, m
geometry_msgs/Vector3 direction  # marker facing direction: its +Z axis (unit vector)
```

`ArucoMarkerArray.msg`
```
std_msgs/Header header           # copied from the image (timestamp + camera frame)
ArucoMarker[] markers
```

## 3. Bridges: drivers contract → our internal topics

Run in our container only under `DRIVER_STACK=v2`
(`launch_decentralized_agent.py` and `launch_centralized_swarm.py`, hardware path only).

| Bridge | In | Out | Mapping |
|---|---|---|---|
| `swarm_marvelmind/gnss_bridge` | `gnss/position` | `mm_pos_unf` (`marvelmind_ros2_msgs/HedgePositionAddressed`) | `address` = our `BEACON_ADDR`; `timestamp_ms` = header stamp in ms (new per fix, passes pose_kf's fresh gate); `x/y/z_m` copied; `flags` 0. One message per fix, no timer republish |
| `swarm_aruco/aruco_bridge` | `aruco/landmarks`, `aruco/markers` | `landmarks_unf`, `targets_unf` (`swarm_interfaces/Markers`) | body frame, cm: `x = 100·z·sx`, `y = −100·x·sy`; normal = `(d.x, d.z)` of `direction`, flipped if `d.z < 0`, normalised (`[right, forward]`); NaN poses and poses beyond `max_range` dropped; `ids` kept in order; header copied, empty frames forwarded |

Scale and range per marker set come from `LANDMARK_SCALE/RANGE` and `TARGET_SCALE/RANGE` at the
top of each launch file (shared with our own detectors). Tests: `swarm_aruco/test/test_aruco_bridge.py`,
`swarm_marvelmind/test/test_gnss_bridge.py`.

## 4. Marker sets

| Set | Dictionary | Side length | Drivers param | Our max range |
|---|---|---|---|---|
| Landmarks | `DICT_4X4_100` | 0.0356 m | `landmark_dictionary`, `landmark_marker_length` | 2.5 m |
| Targets | `DICT_5X5_50` | 0.0576 m | `dictionary`, `marker_length` | 4.0 m |

## 5. Deployment contract

| | Drivers container | Control container (ours) |
|---|---|---|
| Name | `<host>-drivers` | `<host>` |
| Image | `drivers_image` | `swarm_image` |
| Network / IPC | `host` / `host` | `host` / `host` (shared `/dev/shm` for Fast DDS shared memory) |
| Privileged | yes, `/dev:/dev` | yes |
| Env | `ROBOT_NAME`, `SWARM_NODES`, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `ROS_LOCALHOST_ONLY`, `OMP/OPENBLAS_NUM_THREADS` | as before, plus `DRIVER_STACK=v2` |
| Nodes | `SWARM_NODES = chassis,gnss,aruco_camera_2in1,health` | everything except our drivers |
| Stop | SIGINT (image `STOPSIGNAL`), 30 s timeout | unchanged |

Both containers must use the same `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` (`rmw_fastrtps_cpp`) and
`ROS_LOCALHOST_ONLY`. All come from `vars.yml`.

**Device ownership.** Exactly one process opens each device:

| Device | Owner |
|---|---|
| RoboMaster SDK link (rndis, `usb0`, 192.168.42.2) | drivers: chassis |
| Marvelmind hedgehog (`/dev/serial/by-id/*Marvelmind*`, opened exclusive) | drivers: gnss |
| USB camera (`/dev/v4l/by-id/*-video-index0`) | drivers: aruco_camera_2in1 |
| Contact switch GPIO | ours: contact_switch_node |

**Config overlay** (`swarm_ansible_deploy/drivers_config/`), bind-mounted over the image's
`/root/ws/config/`:

| Ours | Image path | Mode |
|---|---|---|
| `swarm_config.yaml.j2`, rendered from our `swarm_bringup/config/swarm_config.yaml` | `swarm_config.yaml` | ro |
| `params/aruco_camera_2in1.yaml`: targets `DICT_5X5_50`, `max_rate: 15.0` | `params/aruco_camera_2in1.yaml` | ro |
| `calibration/<serial>/<serial>_usb_cam_cal_640x480.yaml`, seeded only if missing on the Pi | `calibration/<serial>/` | rw |

Everything else (other params files, URDF, beacon offsets) comes from the image.

## Known gaps at 1.0.0

- `mode_supervisor` waits 0.5 s for `engage_wheels`; the drivers chassis may take up to 3 s to
  ack. The call can be reported as timed out while it still succeeds.
- No `mm_imu` (Marvelmind IMU) under v2. pose_kf runs without it.
- No LED greeting or sound: our launch skips them under v2.
- Calibration files exist only for RM1 (`159CKC50070ECX`) and RM3 (`159CG9J0050797`). Other robots
  publish IDs with NaN poses (dropped by the bridge) until calibrated.
- Our ArUco scale corrections (landmarks 1.0/1.0, targets 1.15/0.8) were tuned for our old camera
  calibration. Re-check them with a tape measure on the USB cameras.

## Upgrade checklist

1. Pick the new upstream tag (`app-<sha>-drivers`) and diff upstream `<old>..<new>` for
   `src/swarm_interfaces`, `src/swarm_chassis`, `src/swarm_gnss`, `src/swarm_aruco`,
   `src/swarm_bringup/launch/robot.launch.py`, `config/params/`.
2. `cmp` both `.msg` files against ours. Different → major bump, re-copy, rebuild our image.
3. Check every row of sections 1, 4 and 5: names, types, QoS, frames, units, stamps, env, paths.
4. Diff upstream `config/params/aruco_camera_2in1.yaml` against our overlay and merge new keys.
5. Write `driver-contract-vXYZ.md` (copy this file), add a changelog line, update `drivers_image`.
6. Deploy to one robot, run the checks in `tasks/validate.yml`, then
   `ros2 topic hz /RMx/odom /RMx/mm_pos_unf /RMx/landmarks_unf` inside our container.

## Changelog

| Version | Date | Drivers image | Change |
|---|---|---|---|
| 1.0.0 | 2026-11-26 | `app-4be73e1-drivers` | First contract: chassis, GNSS, USB camera + ArUco in the drivers container; gnss_bridge and aruco_bridge on our side |
