# Drivers container config overlay

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
Interface contract: `docs/contracts/driver-contract-v100.md`.

## Deploy a robot (example: RM3, `robot3`, 192.168.50.103)

1. **Dev PC:** `git switch split-containers` (rsync copies whatever is checked out).
2. **Build our image on the Pi** (`swarm_docker_img/docker-run.txt`, "Build native on PI"): rsync to
   `robomaster3@192.168.50.103`, then on the Pi:
   ```bash
   cd ~/ros2_humble_swarm && sudo docker login
   sudo docker buildx build -f swarm_docker_img/Dockerfile.tar.arm64 -t elghaliasri/ros2-humble-swarm:robot-arm64-v9.6.0-split --load .
   sudo docker push elghaliasri/ros2-humble-swarm:robot-arm64-v9.6.0-split
   ```
3. **Dev PC:** set `swarm_image` to that tag in `vars.yml`, commit with `./push-split.sh "..."`.
4. **Robot:** USB camera plugged, `vcgencmd get_throttled` = `0x0`, `usb0` link provisioned
   (`usb_link.yml`), calibration in `calibration/<serial>/`.
5. **Deploy** (pulls both images; drivers container `robot3-drivers` starts before ours):
   ```bash
   ansible-playbook -i swarm_ansible_deploy/inventory.ini swarm_ansible_deploy/deploy.yml -l robot3
   ```
   Config/launch-only changes later: `deploy-config.yml`, no rebuild.
6. **Check on the Pi:** `docker ps` shows `robot3` and `robot3-drivers`; `docker logs -f robot3-drivers`
   shows chassis connected, GNSS fix, calibration loaded; inside ours
   (`docker exec -it robot3 bash -c "source /ros2_ws/install/setup.bash && ros2 topic hz /RM3/odom"`):
   `/RM3/odom` ~50 Hz, `/RM3/mm_pos_unf` ~6 Hz, `/RM3/landmarks_unf` <= 15 Hz (republished by
   `perception`), `/RM3/pose` and `/RM3/landmarks` ~20 Hz. Then `ros2 daemon stop`.

**Rollback:** this branch has no single-container mode. Go back with the `hil` branch: `git switch hil`,
set its `swarm_image`, and run its `deploy.yml` (remove `robot3-drivers` first: `docker rm -f robot3-drivers`).
