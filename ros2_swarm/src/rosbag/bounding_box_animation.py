#!/usr/bin/env python3

"""
Animate a RoboMaster / virtual-spacecraft experiment from a ROS 2 sqlite3 bag.

Examples
--------
# Interactive animation:
python3 animate_bag.py ~/rosbags/swarm_20260909_185500 --robot RM1

# Include raw Marvelmind GPS:
python3 animate_bag.py ~/rosbags/swarm_20260909_185500 --robot RM1 --gps

# Include GPS + both IMU headings:
python3 animate_bag.py ~/rosbags/swarm_20260909_185500 --robot RM1 --gps --imu --mm-imu

# Save GIF:
python3 animate_bag.py ~/rosbags/swarm_20260909_185500 --robot RM1 --gif experiment.gif

# Faster playback + GIF:
python3 animate_bag.py ~/rosbags/swarm_20260909_185500 --robot RM1 \
    --gps --imu --mm-imu --speed 2 --gif experiment.gif
"""

import argparse
import math
import sqlite3
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import FancyArrowPatch, Polygon

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# =====================================================================
# ROS BAG
# =====================================================================

def db_files(path):
    path = Path(path).expanduser()

    if path.is_file():
        return [path]

    files = sorted(path.glob("*.db3"))

    if not files:
        raise RuntimeError(f"No .db3 files found in {path}")

    return files


def read_topic(bag, topic):
    """Read one topic from all sqlite3 bag chunks."""
    samples = []

    for db in db_files(bag):
        con = sqlite3.connect(db)

        row = con.execute(
            "SELECT id, type FROM topics WHERE name=?",
            (topic,),
        ).fetchone()

        if row:
            topic_id, msg_type = row
            Msg = get_message(msg_type)

            for timestamp, data in con.execute(
                "SELECT timestamp, data FROM messages "
                "WHERE topic_id=? ORDER BY timestamp",
                (topic_id,),
            ):
                samples.append(
                    (timestamp * 1e-9, deserialize_message(data, Msg))
                )

        con.close()

    return sorted(samples, key=lambda sample: sample[0])


# =====================================================================
# DATA EXTRACTION
# =====================================================================

def quaternion_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def odom_arrays(samples):
    if not samples:
        return tuple(np.array([]) for _ in range(4))

    t = np.asarray([stamp for stamp, _ in samples])
    x = np.asarray([msg.pose.pose.position.x for _, msg in samples])
    y = np.asarray([msg.pose.pose.position.y for _, msg in samples])
    yaw = np.unwrap([quaternion_yaw(msg.pose.pose.orientation) for _, msg in samples])

    return t, x, y, yaw


def wrench_arrays(samples):
    if not samples:
        return tuple(np.array([]) for _ in range(4))

    return (
        np.asarray([stamp for stamp, _ in samples]),
        np.asarray([msg.force.x for _, msg in samples]),
        np.asarray([msg.force.y for _, msg in samples]),
        np.asarray([msg.torque.z for _, msg in samples]),
    )


def gps_arrays(samples):
    if not samples:
        return tuple(np.array([]) for _ in range(3))

    return (
        np.asarray([stamp for stamp, _ in samples]),
        np.asarray([msg.x_m for _, msg in samples]),
        np.asarray([msg.y_m for _, msg in samples]),
    )


# =====================================================================
# HELPERS
# =====================================================================

def interp(t, ts, values):
    return np.interp(t, ts, values)


def latest(t, ts, values, timeout=None):
    """Return latest received value at/before t, optionally enforcing freshness."""
    if len(ts) == 0:
        return None

    i = np.searchsorted(ts, t, side="right") - 1

    if i < 0:
        return None

    if timeout is not None and t - ts[i] > timeout:
        return None

    return values[i]


def triangle(x, y, yaw, size):
    """Same basic triangle geometry as the RViz visualizer."""
    local = np.asarray([
        [size, 0.0],
        [-0.65 * size,  0.60 * size],
        [-0.65 * size, -0.60 * size],
    ])

    c, s = math.cos(yaw), math.sin(yaw)
    R = np.asarray([[c, -s], [s, c]])

    return local @ R.T + np.asarray([x, y])


# =====================================================================
# MAIN
# =====================================================================

parser = argparse.ArgumentParser()

parser.add_argument("bag", help="ROS 2 bag directory or .db3 file")
parser.add_argument("--robot", default="RM1")

parser.add_argument("--gps", action="store_true", help="Show Marvelmind GPS")
parser.add_argument("--imu", action="store_true", help="Show chassis IMU heading")
parser.add_argument("--mm-imu", action="store_true", help="Show Marvelmind IMU heading")
parser.add_argument("--trails", action="store_true", help="Show trajectory history")

parser.add_argument("--gif", help="Optional GIF output filename")
parser.add_argument("--no-show", action="store_true")
parser.add_argument("--fps", type=float, default=20.0)
parser.add_argument("--speed", type=float, default=1.0)

parser.add_argument("--x-min", type=float, default=-1.0)
parser.add_argument("--x-max", type=float, default=1.0)
parser.add_argument("--y-min", type=float, default=-1.0)
parser.add_argument("--y-max", type=float, default=1.0)

parser.add_argument("--triangle-size", type=float, default=0.12)

# Same ideas as your RViz visualizer
parser.add_argument("--maximum-force", type=float, default=1.0)
parser.add_argument("--maximum-torque", type=float, default=0.2)
parser.add_argument("--force-arrow-max-length", type=float, default=1.0)
parser.add_argument("--torque-arrow-max-radius", type=float, default=0.25)

parser.add_argument("--gps-timeout", type=float, default=1.0)
parser.add_argument("--imu-timeout", type=float, default=0.5)

args = parser.parse_args()


# =====================================================================
# TOPICS
# =====================================================================

prefix = f"/{args.robot}"

topics = {
    "virtual": f"{prefix}/virtual_spacecraft/odom",
    "physical": f"{prefix}/localization/odom",
    "wrench": f"{prefix}/virtual_spacecraft/applied_wrench",
    "gps": f"{prefix}/mm_pos_unf",
    "imu": f"{prefix}/localization/imu_odom",
    "mm_imu": f"{prefix}/localization/mm_imu_odom",
}


print(f"Loading {args.robot} from {args.bag}")

virtual = read_topic(args.bag, topics["virtual"])
physical = read_topic(args.bag, topics["physical"])
wrench = read_topic(args.bag, topics["wrench"])

# Heading arrows use GPS as their anchor, just like your RViz node.
need_gps = args.gps or args.imu or args.mm_imu

gps = read_topic(args.bag, topics["gps"]) if need_gps else []
imu = read_topic(args.bag, topics["imu"]) if args.imu else []
mm_imu = read_topic(args.bag, topics["mm_imu"]) if args.mm_imu else []


if not virtual:
    raise RuntimeError(f"No messages found on {topics['virtual']}")

if not physical:
    print(f"WARNING: no physical odometry: {topics['physical']}")

if need_gps and not gps:
    print(f"WARNING: no GPS messages: {topics['gps']}")

if args.imu and not imu:
    print(f"WARNING: no chassis IMU messages: {topics['imu']}")

if args.mm_imu and not mm_imu:
    print(f"WARNING: no Marvelmind IMU messages: {topics['mm_imu']}")


# =====================================================================
# CONVERT
# =====================================================================

tv, xv, yv, yaw_v = odom_arrays(virtual)
tp, xp, yp, yaw_p = odom_arrays(physical)

tw, fx, fy, tz = wrench_arrays(wrench)

tg, xg, yg = gps_arrays(gps)

ti, _, _, yaw_i = odom_arrays(imu)
tm, _, _, yaw_m = odom_arrays(mm_imu)


# =====================================================================
# ANIMATION TIMELINE
# =====================================================================

start = tv[0]
end = tv[-1]

# speed=2 means 2 seconds of experiment time per 1 second of video
dt = args.speed / args.fps

frame_times = np.arange(start, end, dt)

print(f"Experiment duration : {end - start:.2f} s")
print(f"Animation frames    : {len(frame_times)}")
print(f"Playback speed      : {args.speed:.2f}x")


# =====================================================================
# FIGURE
# =====================================================================

fig, ax = plt.subplots(figsize=(9, 8))

ax.set_xlim(args.x_min - 0.15, args.x_max + 0.15)
ax.set_ylim(args.y_min - 0.15, args.y_max + 0.15)
ax.set_aspect("equal")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.grid(True)


# Bounding box
ax.plot(
    [
        args.x_min,
        args.x_max,
        args.x_max,
        args.x_min,
        args.x_min,
    ],
    [
        args.y_min,
        args.y_min,
        args.y_max,
        args.y_max,
        args.y_min,
    ],
    linewidth=2,
    label="Search region",
)


# Robot triangles
virtual_triangle = Polygon(
    np.zeros((3, 2)),
    closed=True,
    fill=False,
    linewidth=2.5,
    label="Virtual spacecraft",
)

physical_triangle = Polygon(
    np.zeros((3, 2)),
    closed=True,
    fill=False,
    linewidth=2.5,
    alpha=0.65,
    label="Physical robot",
)

ax.add_patch(virtual_triangle)
ax.add_patch(physical_triangle)


# Applied force arrow
force_arrow = FancyArrowPatch(
    (0, 0),
    (0, 0),
    arrowstyle="-|>",
    mutation_scale=18,
    linewidth=2,
)

ax.add_patch(force_arrow)


# Torque arrow
torque_arrow = FancyArrowPatch(
    (0, 0),
    (0, 0),
    arrowstyle="-|>",
    connectionstyle="arc3,rad=0.7",
    mutation_scale=16,
    linewidth=2,
)

ax.add_patch(torque_arrow)


# GPS
gps_point = ax.scatter([], [], s=55, marker="o", label="Marvelmind GPS")


# Heading arrows
imu_arrow = FancyArrowPatch(
    (0, 0),
    (0, 0),
    arrowstyle="-|>",
    mutation_scale=15,
    linewidth=2,
)

mm_imu_arrow = FancyArrowPatch(
    (0, 0),
    (0, 0),
    arrowstyle="-|>",
    mutation_scale=15,
    linewidth=2,
)

ax.add_patch(imu_arrow)
ax.add_patch(mm_imu_arrow)


# Optional trajectory trails
virtual_trail, = ax.plot([], [], "--", linewidth=1.2, alpha=0.6)
physical_trail, = ax.plot([], [], "-", linewidth=1.2, alpha=0.6)


title = ax.set_title("")


# Hide optional artists initially
gps_point.set_visible(False)
imu_arrow.set_visible(False)
mm_imu_arrow.set_visible(False)

if not args.trails:
    virtual_trail.set_visible(False)
    physical_trail.set_visible(False)


# =====================================================================
# UPDATE
# =====================================================================

def update(frame):
    t = frame_times[frame]

    # ---------------------------------------------------------
    # Virtual spacecraft
    # ---------------------------------------------------------

    vx = interp(t, tv, xv)
    vy = interp(t, tv, yv)
    vyaw = interp(t, tv, yaw_v)

    virtual_triangle.set_xy(
        triangle(vx, vy, vyaw, args.triangle_size)
    )


    # ---------------------------------------------------------
    # Physical RoboMaster
    # ---------------------------------------------------------

    if len(tp) and tp[0] <= t <= tp[-1]:
        px = interp(t, tp, xp)
        py = interp(t, tp, yp)
        pyaw = interp(t, tp, yaw_p)

        physical_triangle.set_xy(
            triangle(px, py, pyaw, args.triangle_size)
        )

        physical_triangle.set_visible(True)

    else:
        physical_triangle.set_visible(False)


    # ---------------------------------------------------------
    # Applied force
    #
    # This is already inertial/global frame in your RViz code,
    # so DO NOT rotate it by spacecraft yaw.
    # ---------------------------------------------------------

    if len(tw) and tw[0] <= t <= tw[-1]:

        fxi = interp(t, tw, fx)
        fyi = interp(t, tw, fy)

        scale = args.force_arrow_max_length / args.maximum_force

        force_arrow.set_positions(
            (vx, vy),
            (
                vx + scale * fxi,
                vy + scale * fyi,
            ),
        )

        force_arrow.set_visible(math.hypot(fxi, fyi) > 1e-8)

    else:
        force_arrow.set_visible(False)


    # ---------------------------------------------------------
    # Torque
    # ---------------------------------------------------------

    if len(tw) and tw[0] <= t <= tw[-1]:

        torque = interp(t, tw, tz)

        if abs(torque) > 1e-8:

            direction = 1.0 if torque >= 0.0 else -1.0

            radius = (
                args.torque_arrow_max_radius
                * abs(torque)
                / args.maximum_torque
            )

            start_angle = vyaw - direction * 0.7 * math.pi
            end_angle = vyaw + direction * 0.7 * math.pi

            p1 = (
                vx + radius * math.cos(start_angle),
                vy + radius * math.sin(start_angle),
            )

            p2 = (
                vx + radius * math.cos(end_angle),
                vy + radius * math.sin(end_angle),
            )

            torque_arrow.set_positions(p1, p2)
            torque_arrow.set_connectionstyle(
                f"arc3,rad={0.7 * direction}"
            )

            torque_arrow.set_visible(True)

        else:
            torque_arrow.set_visible(False)

    else:
        torque_arrow.set_visible(False)


    # ---------------------------------------------------------
    # GPS
    # ---------------------------------------------------------

    gx = latest(t, tg, xg, args.gps_timeout)
    gy = latest(t, tg, yg, args.gps_timeout)

    gps_valid = gx is not None and gy is not None

    if args.gps and gps_valid:
        gps_point.set_offsets([[gx, gy]])
        gps_point.set_visible(True)
    else:
        gps_point.set_visible(False)


    # ---------------------------------------------------------
    # Chassis IMU
    #
    # Same convention as your RViz visualizer:
    # heading arrow is anchored at the GPS position.
    # ---------------------------------------------------------

    iyaw = latest(t, ti, yaw_i, args.imu_timeout)

    if args.imu and gps_valid and iyaw is not None:

        length = 0.20

        imu_arrow.set_positions(
            (gx, gy),
            (
                gx + length * math.cos(iyaw),
                gy + length * math.sin(iyaw),
            ),
        )

        imu_arrow.set_visible(True)

    else:
        imu_arrow.set_visible(False)


    # ---------------------------------------------------------
    # Marvelmind IMU
    # ---------------------------------------------------------

    myaw = latest(t, tm, yaw_m, args.imu_timeout)

    if args.mm_imu and gps_valid and myaw is not None:

        length = 0.28

        mm_imu_arrow.set_positions(
            (gx, gy),
            (
                gx + length * math.cos(myaw),
                gy + length * math.sin(myaw),
            ),
        )

        mm_imu_arrow.set_visible(True)

    else:
        mm_imu_arrow.set_visible(False)


    # ---------------------------------------------------------
    # Optional trails
    # ---------------------------------------------------------

    if args.trails:

        iv = np.searchsorted(tv, t)
        virtual_trail.set_data(xv[:iv], yv[:iv])

        if len(tp):
            ip = np.searchsorted(tp, t)
            physical_trail.set_data(xp[:ip], yp[:ip])


    # ---------------------------------------------------------
    # Time
    # ---------------------------------------------------------

    title.set_text(
        f"{args.robot} — Bounding Box Search   "
        f"t = {t - start:6.2f} s"
    )

    return (
        virtual_triangle,
        physical_triangle,
        force_arrow,
        torque_arrow,
        gps_point,
        imu_arrow,
        mm_imu_arrow,
        virtual_trail,
        physical_trail,
        title,
    )


# =====================================================================
# RUN / SAVE
# =====================================================================

animation = FuncAnimation(
    fig,
    update,
    frames=len(frame_times),
    interval=1000.0 / args.fps,
    blit=False,
)


if args.gif:
    print(f"Saving GIF → {args.gif}")

    animation.save(
        args.gif,
        writer=PillowWriter(fps=args.fps),
        dpi=120,
    )

    print("GIF saved.")


if not args.no_show:
    plt.show()