#!/bin/bash
set -e

echo "================================="
echo "[swarm_robot] Container starting"
echo "================================="

# Wait for network (important for WiFi robots / Pi boot timing)
# until ip addr show | grep -q "inet "; do
#   echo "[swarm_robot] Waiting for IP address..."
#   sleep 2
# done

# Source ROS
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "[swarm_robot] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[swarm_robot] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[swarm_robot] ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "[swarm_robot] ROBOT_IDX=$ROBOT_IDX"
echo "[swarm_robot] LAUNCH_FILE=${LAUNCH_FILE:=bringup.launch.py}"



echo "[swarm_robot] Launching centralized swarm..."
exec ros2 launch swarm_bringup ${LAUNCH_FILE:=bringup.launch.py}
