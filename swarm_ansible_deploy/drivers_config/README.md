# Drivers container config overlay (`DRIVER_STACK=v2`)

Only what we override in the swarm_rover_v2 drivers image; everything else comes from the image.
Ansible (`tasks/drivers_container.yml`) puts these under `/opt/robomaster/drivers_config/` on the Pi and
bind-mounts them over the image's `/root/ws/config/...`:

| Here | In the drivers container |
|---|---|
| `swarm_config.yaml.j2` (rendered from our `swarm_config.yaml`) | `config/swarm_config.yaml` |
| `params/aruco_camera_2in1.yaml` | `config/params/aruco_camera_2in1.yaml` |
| `calibration/<serial>/` (read-write) | `config/calibration/<serial>/` |

Calibration files are only seeded when missing on the Pi, so a calibration made there (his `calibrate`
node, `nodes:=calibrate`) is never overwritten. Copy it back here and commit it to keep it.
When upgrading the image, diff `params/aruco_camera_2in1.yaml` against the new image's copy.
