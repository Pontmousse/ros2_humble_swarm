

###########################################################################################
###########################################################################################
# def _split_env(name, cast=str):
#     val = os.getenv(name)
#     if val is None:
#         raise RuntimeError(f"Missing required environment variable: {name}")
#     return [cast(x.strip()) for x in val.split(",") if x.strip()]

# def load_swarm_config():
#     """
#     Load swarm configuration from resolved environment variables.
#     Supports 1 or more robots via comma-separated ENV vars.
#     """

#     robot_names = _split_env("ROBOT_NAME", str)
#     beacon_addresses = _split_env("BEACON_ADDR", int)
#     robot_serial_numbers = _split_env("ROBOT_SERIAL", str)
#     init_orientations = _split_env("INITIAL_ORIENT", float)

#     n = len(robot_names)

#     if not (
#         len(beacon_addresses) ==
#         len(robot_serial_numbers) ==
#         len(init_orientations) ==
#         n
#     ):
#         raise ValueError("ENV var lists must all have the same length")

#     return (
#         beacon_addresses,
#         robot_names,
#         robot_serial_numbers,
#         init_orientations,
#     )



###########################################################################################
###########################################################################################


# # -----------------------------
# # Load swarm config (host-side, multi-robot)
# # -----------------------------
# if [ -z "$ROBOT_IDX" ]; then
#   echo "ERROR: ROBOT_IDX not set (e.g. \"1,3,4\")"
#   exit 1
# fi

# CONFIG_YAML="/home/$USER/ros2_humble_swarm/scripts/swarm_config.yaml"

# if [ ! -f "$CONFIG_YAML" ]; then
#   echo "ERROR: swarm config not found: $CONFIG_YAML"
#   exit 1
# fi

# # Split ROBOT_IDX (1-based)
# IFS=',' read -ra IDX_LIST <<< "$ROBOT_IDX"

# ROBOT_NAMES=()
# BEACON_ADDRS=()
# ROBOT_SERIALS=()
# INITIAL_ORIENTS=()

# NUM_ROBOTS=$(yq '.robots | length' "$CONFIG_YAML")

# for idx in "${IDX_LIST[@]}"; do
#   idx="$(echo "$idx" | xargs)"   # trim
#   IDX0=$((idx - 1))

#   if [ "$IDX0" -lt 0 ] || [ "$IDX0" -ge "$NUM_ROBOTS" ]; then
#     echo "ERROR: ROBOT_IDX entry $idx out of range"
#     exit 1
#   fi

#   ROBOT_NAMES+=("$(yq -r ".robots[$IDX0].name" "$CONFIG_YAML")")
#   BEACON_ADDRS+=("$(yq -r ".robots[$IDX0].beacon" "$CONFIG_YAML")")
#   ROBOT_SERIALS+=("$(yq -r ".robots[$IDX0].serial" "$CONFIG_YAML")")
#   INITIAL_ORIENTS+=("$(yq -r ".robots[$IDX0].init_orientation" "$CONFIG_YAML")")
# done

# # Join arrays into comma-separated strings
# export ROBOT_NAME=$(IFS=,; echo "${ROBOT_NAMES[*]}")
# export BEACON_ADDR=$(IFS=,; echo "${BEACON_ADDRS[*]}")
# export ROBOT_SERIAL=$(IFS=,; echo "${ROBOT_SERIALS[*]}")
# export INITIAL_ORIENT=$(IFS=,; echo "${INITIAL_ORIENTS[*]}")

# # Sanity print
# echo "Resolved swarm:"
# echo "  ROBOT_NAME=$ROBOT_NAME"
# echo "  BEACON_ADDR=$BEACON_ADDR"
# echo "  ROBOT_SERIAL=$ROBOT_SERIAL"
# echo "  INITIAL_ORIENT=$INITIAL_ORIENT"


###########################################################################################
###########################################################################################