
ros2_humble_swarm
=================

Lightweight workspace to develop and run ROS 2 Humble-based swarm software and related tools (Marvelmind, Robomaster, aruco, controllers, filters, docker images, and deployment automation).

This repository collects multiple related ROS 2 packages and utilities used for building and experimenting with robot swarms. It includes local ROS packages under several subfolders, Docker container files, and an Ansible playbook for deployment.

## Repository layout

- `ros2_marvelmind/` — Marvelmind integration packages and messages.
- `ros2_robomaster/` — Robomaster drivers, messages and ffmpeg image transport.
- `ros2_swarm/` — Swarm packages (bringup, controllers, filters, aruco, kalman, magnet, marvelmind integrations, interfaces).
- `swarm_docker_img/` — Dockerfiles and scripts used to build swarm runtime images.
- `swarm_ansible_deploy/` — Ansible playbook and inventory for remote deployment.
- `docker-compose.yml` — Example compose file to bring up services in containers.

Each of the top-level folders is a ROS 2 workspace (contains `src/`, `build/`, `install/`, `log/`). Builds are performed with `colcon`.

## Requirements

- Ubuntu 22.04 (recommended) or another Linux distro supported by ROS 2 Humble
- ROS 2 Humble installed: https://docs.ros.org/en/humble/Installation.html
- Python 3.10+
- colcon build
- Docker & docker-compose (optional, for containerized runs)
- Ansible (optional, for deploy)

## Quick setup & build

1. Open a terminal and source your ROS 2 Humble installation (example):

```bash
source /opt/ros/humble/setup.bash
```

2. From the repository root, build the workspaces you need. Example: build the `ros2_swarm` workspace:

```bash
cd ros2_swarm
colcon build --symlink-install
source install/local_setup.bash
```

If you need to build multiple workspaces, build each one (or create a super workspace that includes the `src/` of all required packages).

## Running components

- Bring up the swarm bringup launch (example):

```bash
source ros2_swarm/install/local_setup.bash
ros2 launch swarm_bringup bringup.launch.py
```

- Run a single node (example):

```bash
source ros2_swarm/install/local_setup.bash
ros2 run swarm_controller controller_node
```

Adjust package and node names to match the specific package you want to run. Use `ros2 pkg list` and `ros2 node list` for discovery.

## Docker

This repo contains `swarm_docker_img` with Dockerfiles and helper scripts to build runtime images. There is also an example `docker-compose.yml` at the project root for local multi-container setups.

Build an image (example):

```bash
cd swarm_docker_img
docker build -t ros2-swarm:latest -f Dockerfile.src .
```

Start services with docker-compose:

```bash
docker-compose up --build
```

Note: containerized setups typically require mapping devices (serial ports, USB) and configuring network modes if robots are on a separate network.

## Ansible deployment

Use the `swarm_ansible_deploy/deploy.yml` playbook and `inventory.ini` to deploy images and launch scripts to remote hosts.

Example (from repo root):

```bash
cd swarm_ansible_deploy
ansible-playbook -i inventory.ini deploy.yml
```

Customize `inventory.ini` with your hosts and SSH configuration.

## Testing

If packages include tests, run them with pytest/ament as appropriate. For ROS 2 unit tests run:

```bash
colcon test
colcon test-result --verbose
```

## Troubleshooting

- If a package isn't found, ensure you've sourced the correct `install/local_setup.bash`. Use `ros2 pkg list | grep <pkg>` to verify visibility.
- If device permissions block hardware (serial, USB), ensure your user is in the `dialout` or appropriate group and udev rules allow access.
- For Docker: map host devices and enable `--privileged` only if necessary. Check container logs with `docker logs <container>`.

## Contributing

Contributions are welcome. Suggested workflow:

1. Fork the repo and create a feature branch.
2. Add or update packages under the appropriate workspace (`ros2_swarm`, `ros2_marvelmind`, etc.).
3. Add or update tests for new functionality.
4. Submit a pull request with a clear description and testing notes.

## License

Check individual package folders for their license files. If not present, please coordinate with the repo maintainers before reusing code.
