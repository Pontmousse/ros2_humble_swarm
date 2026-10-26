#!/usr/bin/env python3
"""Grab one frame from a ROS Image topic and save it to disk.

Usage:
    ros2 topic ...                      # (have the stream running)
    python3 scripts/save_snapshot.py [topic] [out.png]

Defaults to /RM2/camera/image_color -> snapshot.png. Subscribes BEST_EFFORT so
it matches the camera's sensor-data QoS (and a RELIABLE publisher too).
"""

import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

try:
    import cv2
except ImportError:
    cv2 = None


def save(msg, path):
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding
    if enc == "mono8":
        img = buf.reshape(h, w)
    elif enc in ("rgb8", "bgr8"):
        img = buf.reshape(h, w, 3)
        if enc == "rgb8" and cv2 is not None:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    else:
        img = buf.reshape(h, -1)  # best effort: show whatever is there
    if cv2 is not None:
        cv2.imwrite(path, img)
    else:
        from PIL import Image as PILImage
        PILImage.fromarray(img).save(path)
    print(f"saved {path}  ({w}x{h}, encoding={enc})")


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/RM2/camera/image_color"
    out = sys.argv[2] if len(sys.argv) > 2 else "snapshot.png"

    rclpy.init()
    node = Node("save_snapshot")
    done = {"ok": False}
    qos = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    def cb(msg):
        if not done["ok"]:
            save(msg, out)
            done["ok"] = True

    node.create_subscription(Image, topic, cb, qos)
    print(f"waiting for a frame on {topic} ...")
    import time
    t0 = time.time()
    while rclpy.ok() and not done["ok"] and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not done["ok"]:
        print("no frame received within 15s")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
