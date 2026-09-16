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
LAUNCH_FILE="${LAUNCH_FILE:-launch_decentralized_agent.py}"

echo "[swarm_robot] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[swarm_robot] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[swarm_robot] ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "[swarm_robot] ROBOT_IDX=$ROBOT_IDX"
echo "[swarm_robot] VIRTUAL_SPACECRAFT=$VIRTUAL_SPACECRAFT"
echo "[swarm_robot] LAUNCH_FILE=$LAUNCH_FILE"

echo "[swarm_robot] Launching swarm agent..."

exec ros2 launch swarm_bringup "$LAUNCH_FILE"