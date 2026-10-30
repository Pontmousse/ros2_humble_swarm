#!/bin/bash
set -e

echo "================================="
echo "[swarm_robot] Container starting"
echo "================================="

# Source ROS
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Defaults -- Ansible/Docker environment can override these
VIRTUAL_SPACECRAFT="${VIRTUAL_SPACECRAFT:-true}"
DIRECT_CMD_VEL="${DIRECT_CMD_VEL:-false}"
LAUNCH_FILE="${LAUNCH_FILE:-launch_decentralized_agent.py}"

echo "[swarm_robot] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[swarm_robot] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[swarm_robot] ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "[swarm_robot] ROBOT_IDX=$ROBOT_IDX"
echo "[swarm_robot] VIRTUAL_SPACECRAFT=$VIRTUAL_SPACECRAFT"
echo "[swarm_robot] DIRECT_CMD_VEL=$DIRECT_CMD_VEL"
echo "[swarm_robot] LAUNCH_FILE=$LAUNCH_FILE"
# The driver's own startup banner prints conn_type, but it only runs when
# RM_DIAGNOSTICS != off, which is not the default. Without this line there is
# no record anywhere of which transport a robot came up on -- which is exactly
# how RM2 (rndis) and RM6 (sta) diverged unnoticed. "(unset)" is itself
# meaningful: it means the launch file's built-in default decided, and that
# default is not the same across image versions.
echo "[swarm_robot] CONN_TYPE=${CONN_TYPE:-(unset -> launch-file default)}"

echo "[swarm_robot] Launching swarm agent..."

exec ros2 launch swarm_bringup "$LAUNCH_FILE"