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

def common_t0(*sample_lists):
    """Earliest timestamp across all topics.

    Zeroing each topic to its own first message fabricates a phase shift: the
    smoother has nothing to publish until cmd_vel_raw exists, so it starts
    ~1.8 s late and would be dragged left into apparent anticipation.
    """
    starts = [s[0][0] for s in sample_lists if s]
    return min(starts) if starts else 0.0


def twist_components(samples, t0):
    """Return t, vx, vy, wz from Twist or TwistStamped messages."""

    if not samples:
        return np.array([]), np.array([]), np.array([]), np.array([])

    t, vx, vy, wz = [], [], [], []

    for stamp, msg in samples:

        # Handles both Twist and TwistStamped
        twist = msg.twist if hasattr(msg, "twist") else msg

        t.append(stamp - t0)
        vx.append(twist.linear.x)
        vy.append(twist.linear.y)
        wz.append(twist.angular.z)

    return map(np.asarray, (t, vx, vy, wz))


def odom_xy(samples, t0):
    """Return t, x, y from nav_msgs/Odometry."""

    if not samples:
        return np.array([]), np.array([]), np.array([])

    t = np.asarray([s - t0 for s, _ in samples])
    x = np.asarray([m.pose.pose.position.x for _, m in samples])
    y = np.asarray([m.pose.pose.position.y for _, m in samples])

    return t, x, y


def yaw_from_quaternion(q):
    """Extract planar yaw from a geometry_msgs quaternion."""
    return np.arctan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def odom_state(samples, t0):
    """Return t, x, y, yaw, speed from nav_msgs/Odometry.

    Speed is the twist magnitude, which is identical in the body and inertial
    frames, so it needs no rotation.
    """
    if not samples:
        return (np.array([]),) * 5

    t = np.asarray([s - t0 for s, _ in samples])
    x = np.asarray([m.pose.pose.position.x for _, m in samples])
    y = np.asarray([m.pose.pose.position.y for _, m in samples])
    yaw = np.asarray(
        [yaw_from_quaternion(m.pose.pose.orientation) for _, m in samples]
    )
    speed = np.asarray(
        [
            np.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
            for _, m in samples
        ]
    )
    return t, x, y, yaw, speed


def wrap_angle(a):
    """Wrap angles to [-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


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

t0 = common_t0(raw, smooth, odom, virtual)

tr, vxr, vyr, wzr = twist_components(raw, t0)
ts, vxs, vys, wzs = twist_components(smooth, t0)

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

_, xo, yo = odom_xy(odom, t0)
_, xv, yv = odom_xy(virtual, t0)

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


# ============================================================
# Tracking error: virtual reference vs physical robot
#
# This is the only figure that measures HIL performance. The reference is
# free to leave the bounding box; what matters is that the two poses overlap.
# ============================================================

tp, xp, yp, yawp, speedp = odom_state(odom, t0)
tv, xv2, yv2, yawv, speedv = odom_state(virtual, t0)

if len(tp) and len(tv):

    # Resample the physical trace onto the reference timestamps so the two
    # series are directly comparable despite different publish rates.
    lo = max(tp[0], tv[0])
    hi = min(tp[-1], tv[-1])
    keep = (tv >= lo) & (tv <= hi)
    te = tv[keep]

    error_x = xv2[keep] - np.interp(te, tp, xp)
    error_y = yv2[keep] - np.interp(te, tp, yp)
    error_yaw = wrap_angle(yawv[keep] - np.interp(te, tp, yawp))
    distance = np.hypot(error_x, error_y)

    reference_speed = speedv[keep]
    physical_speed = np.interp(te, tp, speedp)

    median = np.median(distance)
    p95 = np.percentile(distance, 95)

    print("\n--- tracking error (virtual vs physical) ---")
    print(f"  position: median {median:.3f} m  mean {distance.mean():.3f} m  "
          f"p95 {p95:.3f} m  max {distance.max():.3f} m")
    print(f"  yaw     : median {np.median(np.abs(error_yaw)):.3f} rad  "
          f"p95 {np.percentile(np.abs(error_yaw), 95):.3f} rad")
    print(f"  within 0.10 m for {100 * (distance < 0.10).mean():.1f}% "
          f"of the run")

    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(11, 9))

    ax[0].plot(te, distance, label="|position error|")
    ax[0].axhline(median, color="tab:green", ls=":",
                  label=f"median {median:.3f} m")
    ax[0].axhline(p95, color="tab:red", ls=":", label=f"p95 {p95:.3f} m")
    ax[0].set_ylabel("error [m]")
    ax[0].grid()
    ax[0].legend()

    ax[1].plot(te, error_x, label="x")
    ax[1].plot(te, error_y, label="y")
    ax[1].plot(te, error_yaw, label="yaw [rad]")
    ax[1].axhline(0.0, color="k", lw=0.5)
    ax[1].set_ylabel("signed error")
    ax[1].grid()
    ax[1].legend()

    ax[2].plot(te, reference_speed, "--", label="virtual reference")
    ax[2].plot(te, physical_speed, label="physical")
    ax[2].set_ylabel("speed [m/s]")
    ax[2].set_xlabel("Time [s]")
    ax[2].grid()
    ax[2].legend()

    fig.suptitle(f"{ROBOT} — Tracking Error")
    fig.tight_layout()

plt.show()
