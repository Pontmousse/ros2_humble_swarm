"""Wire format for the swarm state broadcast.

Deliberately free of ROS imports so the sniffer can run on a laptop with no
workspace sourced.

Frame contract: x, y, yaw, vx, vy are ALL in `swarm_map`, the shared arena
frame. The sender rotates its body-frame twist before encoding, so a receiver
can dead-reckon with `x + vx*dt` without knowing the sender's heading.
"""
import struct

MAGIC = b"SW"
VERSION = 1

# Little-endian, no padding. Pose/velocity are float32: at a 10 m arena scale
# that still resolves ~1e-6 m, far below the Marvelmind noise floor.
_FMT = "<2sBBIdffffffB"
PACKET_SIZE = struct.calcsize(_FMT)  # 41 bytes

FLAG_POSE_VALID = 1 << 0

DEFAULT_PORT = 9870
DEFAULT_BROADCAST = "192.168.50.255"


def encode(robot_id, seq, t_send, x, y, yaw, vx, vy, omega, pose_valid=True):
    flags = FLAG_POSE_VALID if pose_valid else 0
    return struct.pack(
        _FMT, MAGIC, VERSION, robot_id, seq & 0xFFFFFFFF, t_send,
        x, y, yaw, vx, vy, omega, flags,
    )


def decode(data):
    """Return a dict, or None if the datagram is not ours."""
    if len(data) != PACKET_SIZE:
        return None
    magic, version, robot_id, seq, t_send, x, y, yaw, vx, vy, omega, flags = \
        struct.unpack(_FMT, data)
    if magic != MAGIC or version != VERSION:
        return None
    return {
        "robot_id": robot_id,
        "seq": seq,
        "t_send": t_send,
        "x": x, "y": y, "yaw": yaw,
        "vx": vx, "vy": vy, "omega": omega,
        "pose_valid": bool(flags & FLAG_POSE_VALID),
    }
