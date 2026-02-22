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
install_if_missing gpiod
install_if_missing yq

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
IMAGE="elghaliasri/ros2-humble-swarm:robot-arm64-v8"

# Stop only if running
if docker ps --format '{{.Names}}' | grep -q '^swarm$'; then
  echo ""
  echo "Stopping existing swarm container"
  docker stop swarm
fi

# Remove if exists
if docker ps -a --format '{{.Names}}' | grep -q '^swarm$'; then
  echo ""
  echo "Removing existing swarm container"
  docker rm swarm
fi


if ! sudo docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "Pulling Docker image"
  sudo docker pull "$IMAGE"
else
  echo "Docker image already present"
fi

docker image prune -f


# -----------------------------
# 7. Headless Wi-Fi fixes (safe to repeat)
# -----------------------------
echo "[BOOTSTRAP] Ensuring headless Wi-Fi reliability..."

### 1️⃣ Disable Wi-Fi power saving (NetworkManager)
NM_WIFI_CONF="/etc/NetworkManager/conf.d/wifi-powersave.conf"

if [ ! -f "$NM_WIFI_CONF" ]; then
  echo "[BOOTSTRAP] Disabling Wi-Fi power saving"
  sudo tee "$NM_WIFI_CONF" > /dev/null <<EOF
[connection]
wifi.powersave = 2
EOF
else
  echo "[BOOTSTRAP] Wi-Fi power saving already configured"
fi


### 2️⃣ Set Wi-Fi regulatory domain (safe on Pi 4 + Pi 5)
# Change US to your country code
WIFI_COUNTRY="US"

if command -v iw >/dev/null 2>&1; then
  CURRENT_REG=$(iw reg get | grep country | awk '{print $2}' | tr -d ':')
  if [ "$CURRENT_REG" != "$WIFI_COUNTRY" ]; then
    echo "[BOOTSTRAP] Setting Wi-Fi regulatory domain to $WIFI_COUNTRY"
    sudo iw reg set "$WIFI_COUNTRY" || true
  fi
fi

# Persist it
CRDA_FILE="/etc/default/crda"
if [ -f "$CRDA_FILE" ]; then
  sudo sed -i "s/^REGDOMAIN=.*/REGDOMAIN=$WIFI_COUNTRY/" "$CRDA_FILE"
fi


### 3️⃣ Install a Wi-Fi kick service (only if NetworkManager is present)
if systemctl is-enabled NetworkManager >/dev/null 2>&1; then
  WIFI_KICK_SERVICE="/etc/systemd/system/wifi-kick.service"

  if [ ! -f "$WIFI_KICK_SERVICE" ]; then
    echo "[BOOTSTRAP] Installing Wi-Fi kick service"

    sudo tee "$WIFI_KICK_SERVICE" > /dev/null <<EOF
[Unit]
Description=Force Wi-Fi reconnect after boot
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/bin/nmcli networking off
ExecStart=/bin/sleep 2
ExecStart=/usr/bin/nmcli networking on

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reexec
    sudo systemctl daemon-reload
    sudo systemctl enable wifi-kick.service
  else
    echo "[BOOTSTRAP] Wi-Fi kick service already installed"
  fi
fi


### 4️⃣ Restart NetworkManager (safe)
if systemctl is-active NetworkManager >/dev/null 2>&1; then
  echo "[BOOTSTRAP] Restarting NetworkManager"
  sudo systemctl restart NetworkManager
fi

echo "[BOOTSTRAP] Headless Wi-Fi fixes applied"


# -----------------------------
# 8. Load swarm config (host-side)
# -----------------------------
if [ -z "$ROBOT_IDX" ]; then
  echo "ERROR: ROBOT_IDX not set."
  echo "Set it with Ansible inventory or manually:"
  echo "  echo 'ROBOT_IDX=N' | sudo tee -a /etc/environment"
  echo "Then reboot for the changes to take effect."
  echo "or run:"
  echo "  export ROBOT_IDX=4"
  echo "to apply immediately (but won't persist across reboots)"
  exit 1
fi

echo "ROBOT_IDX=$ROBOT_IDX"


CONFIG_YAML="/home/$USER/ros2_humble_swarm/scripts/swarm_config.yaml"

if [ ! -f "$CONFIG_YAML" ]; then
  echo "ERROR: swarm config not found: $CONFIG_YAML"
  exit 1
fi

# Extract robot-specific values (1-based → 0-based)
IDX0=$((ROBOT_IDX - 1))

ROBOT_NAME=$(yq -r ".robots[$IDX0].name" "$CONFIG_YAML")
BEACON_ADDR=$(yq -r ".robots[$IDX0].beacon" "$CONFIG_YAML")
ROBOT_SERIAL=$(yq -r ".robots[$IDX0].serial" "$CONFIG_YAML")
INITIAL_ORIENT=$(yq -r ".robots[$IDX0].init_orientation" "$CONFIG_YAML")


# Sanity check
if [ -z "$ROBOT_NAME" ] || [ "$ROBOT_NAME" = "null" ]; then
  echo "ERROR: Invalid ROBOT_IDX=$ROBOT_IDX for swarm config"
  exit 1
fi

echo "Resolved robot:"
echo "  NAME=$ROBOT_NAME"
echo "  BEACON=$BEACON_ADDR"
echo "  SERIAL=$ROBOT_SERIAL"
echo "  INIT_ORIENT=$INITIAL_ORIENT"
echo ""


# -----------------------------
# 9. Run container IF not running
# -----------------------------
: "${LAUNCH_FILE:=launch_magnet.py}"
export LAUNCH_FILE


CONTAINER="swarm"

if ! sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "Starting swarm container"
  sudo docker run -d \
    -e LAUNCH_FILE=$LAUNCH_FILE \
    -e ROBOT_NAME="$ROBOT_NAME" \
    -e BEACON_ADDR="$BEACON_ADDR" \
    -e ROBOT_SERIAL="$ROBOT_SERIAL" \
    -e INITIAL_ORIENT="$INITIAL_ORIENT" \
    -e GPIO_LINE=4 \
    --name "$CONTAINER" \
    --restart always \
    --privileged \
    --network host \
    --device /dev/gpiomem \
    --device /dev/gpiochip0 \
    --device /dev/gpiochip4 \
    --device /dev/ttyUSB0 \
    --device /dev/ttyUSB1 \
    --device /dev/ttyAMA0 \
    "$IMAGE"
else
  echo "Swarm container already running"
fi

echo "=== Bootstrap complete ==="
echo "Reboot REQUIRED to apply group permissions"
