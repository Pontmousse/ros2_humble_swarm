#!/usr/bin/env bash
# Record a focused rosbag for post-investigation of the pose_kf localization
# loop on hardware. Captures the KF's raw inputs, its outputs, and the
# controller signals needed to correlate any instability -- but NOT camera
# images (too big; add them with RECORD_IMAGES=1 if you really need them).
#
# Usage:
#   ./scripts/record_kf_debug.sh                 # robots RM2 (default), auto name
#   ROBOTS="RM1 RM2" ./scripts/record_kf_debug.sh
#   BAG=myrun ./scripts/record_kf_debug.sh       # fixed bag name
#   RECORD_IMAGES=1 ./scripts/record_kf_debug.sh # also grab camera topics
#
# Stop with Ctrl-C. Bags land in ./bags/.
set -euo pipefail

ROBOTS="${ROBOTS:-RM2}"
BAG="${BAG:-kf_debug_$(date +%Y%m%d_%H%M%S)}"
OUTDIR="${OUTDIR:-bags}"
mkdir -p "$OUTDIR"

# Per-robot topics to record (namespace-relative, prefixed below).
PER_ROBOT=(
  # --- KF raw inputs ---
  mm_pos_unf          # Marvelmind fixes (fresh-gated by the KF)
  odom                # driver body twist + absolute yaw quaternion
  imu mm_imu          # gyro (whichever the driver/marvelmind provides)
  cmd_vel             # prediction input u
  # --- KF outputs (the contract) ---
  pose                # State: x, y, theta(deg)
  localization/odom   # fused pose + BODY-frame velocity + omega
  # --- controller signals, to correlate instability ---
  landmarks targets   # guidance / chase inputs
  virtual_spacecraft/odom   # chase velocity servo reference
)

TOPICS=()
for r in $ROBOTS; do
  for t in "${PER_ROBOT[@]}"; do
    TOPICS+=("/$r/$t")
  done
done
# Global frames.
TOPICS+=(/tf /tf_static)

if [[ "${RECORD_IMAGES:-0}" == "1" ]]; then
  for r in $ROBOTS; do
    TOPICS+=("/$r/camera/image_raw" "/$r/image_raw")
  done
fi

echo "Recording -> $OUTDIR/$BAG"
echo "Robots    : $ROBOTS"
echo "Topics    :"; printf '  %s\n' "${TOPICS[@]}"
echo

# --use-sim-time off (hardware wall clock). Missing topics are skipped with a
# warning, so the same script works whether chase or encapsulation is running.
exec ros2 bag record -o "$OUTDIR/$BAG" "${TOPICS[@]}"
