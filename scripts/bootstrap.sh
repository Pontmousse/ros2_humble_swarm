#!/usr/bin/env bash
set -e

echo "=== RoboMaster Pi bootstrap (host-only, no Python) ==="

# -----------------------------
# 1. Update apt (safe to repeat)
# -----------------------------
sudo apt update

# -----------------------------
# 2. Install system packages IF missing
# -----------------------------
install_if_missing() {
  if ! dpkg -s "$1" >/dev/null 2>&1; then
    echo "Installing $1"
    sudo apt install -y "$1"
  else
    echo "$1 already installed"
  fi
}

install_if_missing docker.io
install_if_missing git
install_if_missing libopus-dev

# -----------------------------
# 3. Enable Docker
# -----------------------------
sudo systemctl enable docker
sudo systemctl start docker

# -----------------------------
# 4. Groups (safe if repeated)
# -----------------------------
sudo groupadd -f gpio
sudo groupadd -f dialout

sudo usermod -aG docker "$USER"
sudo usermod -aG gpio "$USER"
sudo usermod -aG dialout "$USER"

# -----------------------------
# 5. Udev rules (overwrite-safe)
# -----------------------------
UDEV_FILE="/etc/udev/rules.d/99-gpio.rules"

if [ ! -f "$UDEV_FILE" ]; then
  echo "Creating GPIO udev rules"
  sudo tee "$UDEV_FILE" >/dev/null << 'EOF'
SUBSYSTEM=="gpio*", GROUP="gpio", MODE="770"
SUBSYSTEM=="pwm*", GROUP="gpio", MODE="770"
KERNEL=="spidev*", GROUP="gpio", MODE="660"
KERNEL=="i2c-[0-9]*", GROUP="gpio", MODE="660"
KERNEL=="ttyAMA[0-9]*", GROUP="gpio", MODE="660"
KERNEL=="gpiomem", GROUP="gpio", MODE="660"
EOF
else
  echo "Udev rules already exist"
fi

sudo udevadm control --reload-rules
sudo udevadm trigger

# -----------------------------
# 6. Pull Docker image IF missing
# -----------------------------
IMAGE="elghaliasri/ros2-humble-swarm:robot-arm64-v1"

if ! sudo docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "Pulling Docker image"
  sudo docker pull "$IMAGE"
else
  echo "Docker image already present"
fi

# -----------------------------
# 7. Run container IF not running
# -----------------------------
CONTAINER="swarm"

if ! sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "Starting swarm container"
  sudo docker run -d \
    -e LAUNCH_FILE=launch_magnet.py \
    -e ROBOT_IDX=4 \
    --name "$CONTAINER" \
    --restart always \
    --privileged \
    --network host \
    --device /dev/gpiomem \
    --device /dev/ttyUSB0 \
    --device /dev/ttyUSB1 \
    --device /dev/ttyAMA0 \
    "$IMAGE"
else
  echo "Swarm container already running"
fi

echo "=== Bootstrap complete ==="
echo "Reboot REQUIRED to apply group permissions"
