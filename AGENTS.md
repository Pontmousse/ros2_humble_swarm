# Repository Guidelines

## Project Structure

- This repository contains several ROS 2 Humble workspaces: `ros2_swarm/`,
  `ros2_marvelmind/`, and `ros2_robomaster/`.
- ROS packages live under each workspace's `src/` directory.
- `swarm_docker_img/` contains container definitions, and
  `swarm_ansible_deploy/` contains deployment automation.
- Put shared documentation in `docs/`. Keep package-specific files with their
  package.

## Working Conventions

<!-- - This machine does not have ROS 2 installed. Do not run builds, source ROS
  setup files, or invoke ROS/colcon commands on this machine. Limit local
  verification to non-ROS static checks unless the user explicitly provides a
  ROS-capable environment. -->

- Prefer concise, compact code when it remains clear and readable.
- Do not expand simple logic across many lines purely for aesthetics or formatting.
- Simple functions, callbacks, conditionals, and expressions may be written as one-liners when their intent is immediately obvious.
- Avoid excessive vertical spacing, unnecessary intermediate variables, and needless helper functions.
- Do not introduce abstractions solely to make the code look more structured.
- Prefer the simplest readable implementation over a more verbose or "clean-looking" version.
- Match the compactness of the surrounding codebase rather than reformatting existing code into a more expanded style.
- When both versions are equally readable, prefer the version with fewer lines.

- The bounding box in `virtual_spacecraft.yaml` is a guidance target, not a hard
  constraint: rebound firings are force-limited, so the virtual spacecraft is
  expected to overshoot past a wall during a bounce before reversing. Define the
  box once in the top-level `/**` block so guidance and visualization stay in sync.

###############################################################################

- Make focused changes and preserve the existing package layout and style.
- Do not edit generated `build/`, `install/`, or `log/` directories.
- Keep ROS node, topic, frame, parameter, and executable names backward
  compatible unless a rename is explicitly requested.
- When adding a Python ROS node, update the package's `setup.py` entry points
  and dependencies in `package.xml` as needed.
- When adding messages or services, update the relevant `CMakeLists.txt` and
  `package.xml`.
- Avoid hardware assumptions in reusable logic. Expose robot-specific values
  through ROS parameters or configuration files where practical.
- Prefer simple implementations without overthinking.
