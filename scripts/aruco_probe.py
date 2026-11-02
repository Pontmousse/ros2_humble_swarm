#!/usr/bin/env python3
"""Live ArUco dictionary probe.

Subscribes to the robot's camera topic, grabs frames, and runs detectMarkers
against EVERY predefined OpenCV dictionary -- telling you which dictionary your
printed tags actually belong to (or that none match -> resolution/border issue).

Usage (centralized, RoboMaster h264 camera):
    python3 /tmp/aruco_probe.py --topic /RM2/camera/image_h264 --h264

Usage (decentralized / ext_camera mono8):
    python3 /tmp/aruco_probe.py --topic /RM2/camera/image_color
"""
import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2


def all_dicts():
    names = [n for n in dir(cv2.aruco) if n.startswith("DICT_")]
    out = {}
    for n in names:
        try:
            out[n] = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, n))
        except Exception:
            pass
    return out


class Probe(Node):
    def __init__(self, topic, use_h264, frames):
        super().__init__("aruco_probe")
        self.max_frames = frames
        self.seen = 0
        self.hits = {}  # dict_name -> set(ids)
        self.dicts = all_dicts()
        self.params = cv2.aruco.DetectorParameters()
        self.detectors = {
            name: cv2.aruco.ArucoDetector(d, self.params)
            for name, d in self.dicts.items()
        }
        self.get_logger().info(
            f"Testing {len(self.detectors)} dictionaries on {topic}")

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        if use_h264:
            import av
            from robomaster_msgs.msg import H264Packet
            self.decoder = av.codec.CodecContext.create("h264", "r")
            self.create_subscription(H264Packet, topic, self.on_h264, qos)
        else:
            from cv_bridge import CvBridge
            from sensor_msgs.msg import Image
            self.bridge = CvBridge()
            self.create_subscription(Image, topic, self.on_image, qos)

    def on_h264(self, msg):
        import av
        try:
            for packet in self.decoder.parse(bytes(msg.data)):
                try:
                    for frame in self.decoder.decode(packet):
                        self.process(frame.to_ndarray(format="gray"))
                except av.error.InvalidDataError:
                    continue
        except Exception as e:
            self.get_logger().warn(f"decode error: {e}")

    def on_image(self, msg):
        enc = "mono8" if msg.encoding in ("mono8", "8UC1") else "bgr8"
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=enc)
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.process(img)

    def process(self, gray):
        self.seen += 1
        for name, det in self.detectors.items():
            corners, ids, _ = det.detectMarkers(gray)
            if ids is not None and len(ids) > 0:
                self.hits.setdefault(name, set()).update(int(i) for i in ids.flatten())
        if self.seen % 5 == 0 or self.seen >= self.max_frames:
            self.report()
        if self.seen >= self.max_frames:
            raise KeyboardInterrupt

    def report(self):
        print(f"\n===== after {self.seen} frame(s) =====")
        if not self.hits:
            print("NO dictionary matched any marker.")
            print("-> quad not decoding: resolution/focus/blur or missing white")
            print("   border (quiet zone). Move closer / improve focus/light.")
        else:
            for name in sorted(self.hits):
                ids = sorted(self.hits[name])
                print(f"  MATCH  {name:24s} ids={ids}")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True)
    ap.add_argument("--h264", action="store_true",
                    help="topic is a RoboMaster H264Packet stream")
    ap.add_argument("--frames", type=int, default=30,
                    help="stop after this many frames")
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.topic, args.h264, args.frames)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
