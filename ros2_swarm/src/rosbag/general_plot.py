#!/usr/bin/env python3

import sys, sqlite3
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ============================================================
# Configuration
# ============================================================

ROBOT = "RM2"

TOPICS = {
    "raw":      f"/{ROBOT}/cmd_vel_raw",
    "smooth":   f"/{ROBOT}/cmd_vel",
    "odom":     f"/{ROBOT}/localization/odom",
    "virtual":  f"/{ROBOT}/virtual_spacecraft/odom",
}


# ============================================================
# ROS bag reader
# ============================================================

def find_db3(bag_path):
    """Find all sqlite3 chunks inside a rosbag directory."""
    path = Path(bag_path)
    return sorted(path.glob("*.db3")) if path.is_dir() else [path]


def read_topic(bag_path, topic_name):
    """Read and deserialize one topic from a ROS 2 sqlite3 bag."""

    samples = []

    for db_file in find_db3(bag_path):

        con = sqlite3.connect(db_file)
        cur = con.cursor()

        row = cur.execute(
            "SELECT id, type FROM topics WHERE name=?",
            (topic_name,)
        ).fetchone()

        if row is None:
            con.close()
            continue

        topic_id, msg_type = row
        Msg = get_message(msg_type)

        rows = cur.execute(
            "SELECT timestamp, data FROM messages "
            "WHERE topic_id=? ORDER BY timestamp",
            (topic_id,)
        )

        for timestamp, data in rows:
            samples.append(
                (timestamp * 1e-9, deserialize_message(data, Msg))
            )

        con.close()

    return samples


# ============================================================
# Message extraction
# ============================================================

def twist_components(samples):
    """Return t, vx, vy, wz from Twist or TwistStamped messages."""

    if not samples:
        return np.array([]), np.array([]), np.array([]), np.array([])

    t0 = samples[0][0]

    t, vx, vy, wz = [], [], [], []

    for stamp, msg in samples:

        # Handles both Twist and TwistStamped
        twist = msg.twist if hasattr(msg, "twist") else msg

        t.append(stamp - t0)
        vx.append(twist.linear.x)
        vy.append(twist.linear.y)
        wz.append(twist.angular.z)

    return map(np.asarray, (t, vx, vy, wz))


def odom_xy(samples):
    """Return t, x, y from nav_msgs/Odometry."""

    if not samples:
        return np.array([]), np.array([]), np.array([])

    t0 = samples[0][0]

    t = np.asarray([s - t0 for s, _ in samples])
    x = np.asarray([m.pose.pose.position.x for _, m in samples])
    y = np.asarray([m.pose.pose.position.y for _, m in samples])

    return t, x, y


# ============================================================
# Main
# ============================================================

if len(sys.argv) < 2:
    print("Usage: python3 plot_bag.py ~/rosbags/swarm_YYYYMMDD_HHMMSS")
    sys.exit(1)

bag = sys.argv[1]

print(f"Reading bag: {bag}")

raw = read_topic(bag, TOPICS["raw"])
smooth = read_topic(bag, TOPICS["smooth"])
odom = read_topic(bag, TOPICS["odom"])
virtual = read_topic(bag, TOPICS["virtual"])

print(f"raw cmd_vel : {len(raw)}")
print(f"smooth      : {len(smooth)}")
print(f"local odom  : {len(odom)}")
print(f"virtual odom: {len(virtual)}")


# ============================================================
# Velocity comparison
# ============================================================

tr, vxr, vyr, wzr = twist_components(raw)
ts, vxs, vys, wzs = twist_components(smooth)

fig, ax = plt.subplots(3, 1, sharex=True, figsize=(11, 8))

ax[0].plot(tr, vxr, "--", label="raw")
ax[0].plot(ts, vxs, label="smoothed")
ax[0].set_ylabel("vx [m/s]")
ax[0].grid()
ax[0].legend()

ax[1].plot(tr, vyr, "--", label="raw")
ax[1].plot(ts, vys, label="smoothed")
ax[1].set_ylabel("vy [m/s]")
ax[1].grid()
ax[1].legend()

ax[2].plot(tr, wzr, "--", label="raw")
ax[2].plot(ts, wzs, label="smoothed")
ax[2].set_ylabel("wz [rad/s]")
ax[2].set_xlabel("Time [s]")
ax[2].grid()
ax[2].legend()

fig.suptitle(f"{ROBOT} — Velocity Smoother")
fig.tight_layout()


# ============================================================
# XY trajectory comparison
# ============================================================

_, xo, yo = odom_xy(odom)
_, xv, yv = odom_xy(virtual)

plt.figure(figsize=(8, 8))

if len(xo):
    plt.plot(xo, yo, label="Robot localization")

if len(xv):
    plt.plot(xv, yv, "--", label="Virtual spacecraft")

plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title(f"{ROBOT} — Physical vs Virtual Trajectory")
plt.axis("equal")
plt.grid()
plt.legend()

plt.show()