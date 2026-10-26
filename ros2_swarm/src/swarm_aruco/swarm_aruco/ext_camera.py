"""Publish an external camera's luma plane as mono8 on `camera/image_color`.

Drop-in replacement for the RoboMaster's own camera feed ("external" = any
camera that is not the RoboMaster's). `aruco_detector` already asks cv_bridge
for `mono8` and runs detection on grey, so publishing single-channel here
removes a colour conversion from the chain rather than adding one.

Two capture backends, selected by the `source` parameter (default `csi`):

  * `usb`  -- a UVC/V4L2 device (e.g. See3CAM_CU30) opened with OpenCV. The
    See3CAM streams packed 4:2:2 UYVY, in which luma sits on the odd byte of
    every 2-byte pixel; slicing it out is a strided copy (~1 ms/frame on the
    Pi 4) against ~25 ms for the MJPEG decode path. Measured on a Pi 4
    (640x480 UYVY): 23.8 fps end to end at 35% of the 4-core machine.

  * `csi`  -- a MIPI CSI camera (e.g. Raspberry Pi Camera Module 3, IMX708)
    driven through libcamera/Picamera2. The raw CSI node is Bayer, not a
    usable image, so OpenCV/V4L2 cannot be used here; libcamera's ISP
    debayers into YUV420 whose first `height` rows are the luma plane, which
    we slice out the same cheap way. Picamera2 is imported lazily so this
    node still runs on machines without libcamera when `source` is `usb`.

The published interface (topic, encoding, rate cap) is identical for both, so
the detectors and the rest of the graph do not care which backend is active.
"""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

import cv2

# Byte position of luma within each 2-byte packed-422 pixel. YUYV is Y0 U Y1 V
# so luma sits on even bytes; UYVY is U Y0 V Y1 so it sits on odd ones. Reading
# the wrong offset yields a chroma plane that looks like plausible grey mush and
# detects nothing -- a silent failure, so it is worth getting right.
LUMA_OFFSET = {"YUYV": 0, "UYVY": 1}

SOURCES = ("usb", "csi")


class ExtCamera(Node):
    """Grab from a USB (V4L2) or CSI (libcamera) device and republish mono8."""

    def __init__(self):
        super().__init__("ext_camera")

        # Capture backend. Default csi (Pi Camera Module 3); set to usb to use
        # a UVC camera such as the See3CAM. Nothing else needs to change.
        self.declare_parameter("source", "csi")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fourcc", "UYVY")  # usb only
        self.declare_parameter("fps", 30)
        # Publish rate cap. The detectors throttle themselves via detect_max_hz,
        # so pushing frames faster than they can consume only burns CPU and
        # fills the DDS queue. 0 disables the cap.
        self.declare_parameter("publish_max_hz", 10.0)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("qos_depth", 1)
        self.declare_parameter("qos_reliability", "BEST_EFFORT")

        self.source = str(self.get_parameter("source").value).lower()
        self.device = str(self.get_parameter("device").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fourcc = str(self.get_parameter("fourcc").value).upper()
        self.fps = int(self.get_parameter("fps").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        if self.source not in SOURCES:
            raise RuntimeError(
                f"unsupported source {self.source!r}; use one of {SOURCES}")

        publish_max_hz = float(self.get_parameter("publish_max_hz").value)
        self.min_interval = 1.0 / publish_max_hz if publish_max_hz > 0 else 0.0

        qos = QoSProfile(
            depth=int(self.get_parameter("qos_depth").value),
            reliability=(
                ReliabilityPolicy.RELIABLE
                if str(self.get_parameter("qos_reliability").value) == "RELIABLE"
                else ReliabilityPolicy.BEST_EFFORT),
            history=HistoryPolicy.KEEP_LAST,
        )

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "camera/image_color", qos)

        # Backend handles. Only one is ever opened.
        self.cap = None     # cv2.VideoCapture (usb)
        self.picam = None   # Picamera2 (csi)
        if self.source == "usb":
            self._open_usb()
        else:
            self._open_csi()

        self.last_publish = 0.0
        self.read_failures = 0

        # Drive the grab from a timer rather than a `while rclpy.ok()` loop so
        # the executor keeps servicing parameter and lifecycle callbacks. The
        # period is deliberately shorter than the frame interval: the backend
        # read blocks until the next frame, which is what actually paces us.
        self.create_timer(1.0 / max(1, self.fps * 2), self.grab)

    # ----------------------------- USB (V4L2) ------------------------------
    def _open_usb(self):
        if self.fourcc not in LUMA_OFFSET and self.fourcc != "MJPG":
            raise RuntimeError(
                f"unsupported fourcc {self.fourcc}; use UYVY, YUYV or MJPG")

        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"could not open {self.device}")

        # Order matters: the pixel format has to be selected before the frame
        # size, otherwise the driver negotiates the size against the default
        # format and silently keeps a resolution the requested format cannot
        # deliver.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        # Keep latency down: without this the driver hands us whatever is
        # oldest in its queue, which on a throttled consumer is seconds stale.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.fourcc != "MJPG":
            # Hand us the raw payload so we can slice luma out ourselves.
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

        got = "".join(chr((int(cap.get(cv2.CAP_PROP_FOURCC)) >> (8 * i)) & 0xFF)
                      for i in range(4))
        actual = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                  int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        self.get_logger().info(
            f"usb {self.device}: requested {self.width}x{self.height} "
            f"{self.fourcc}@{self.fps} -> got {actual[0]}x{actual[1]} {got}")
        if got != self.fourcc or actual != (self.width, self.height):
            # Not fatal -- a substituted mode still produces usable images, but
            # the calibration will no longer match, so say so loudly.
            self.get_logger().warn(
                "driver substituted a different mode; intrinsics for the "
                "requested mode will NOT apply to these frames")
            self.width, self.height = actual
        self.cap = cap

    def _read_usb(self):
        """Return a mono8 frame from the V4L2 device, or None on failure."""
        ok, raw = self.cap.read()
        if not ok:
            return None
        if self.fourcc == "MJPG":
            return cv2.imdecode(raw.reshape(-1), cv2.IMREAD_GRAYSCALE)
        # The raw buffer comes back as (H, W, 2), (H, W*2) or flat (1, H*W*2)
        # depending on the OpenCV build and the driver. Slicing the flat form
        # directly yields a 1-pixel-tall image that still has the right pixel
        # count, so it looks plausible downstream while detecting nothing.
        # Reshape to the known height first.
        off = LUMA_OFFSET[self.fourcc]
        return np.ascontiguousarray(raw.reshape(self.height, -1)[:, off::2])

    # ----------------------------- CSI (libcamera) -------------------------
    def _open_csi(self):
        # Imported lazily so a usb-only machine without libcamera can still run
        # this node. NOTE: libcamera's Python bindings are Raspberry-Pi-OS only
        # and are NOT present in the Ubuntu 22.04 (ros:humble) container as
        # built today -- the import below will fail there until the image ships
        # libcamera + picamera2. Fail loudly with that context rather than a
        # bare ModuleNotFoundError. Use CAMERA_SOURCE=usb for the See3CAM.
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError(
                "source=csi needs libcamera + picamera2, which are not "
                "installed in this image (Ubuntu base has no libcamera Python "
                "bindings). Install them in the image, or run with "
                "source=usb / CAMERA_SOURCE=usb."
            ) from exc

        self.picam = Picamera2()
        # YUV420 so the first `height` rows of the returned array are the luma
        # plane -- the same cheap strided slice as the UYVY path, and it avoids
        # a full debayer-to-RGB that ArUco would immediately undo.
        config = self.picam.create_video_configuration(
            main={"size": (self.width, self.height), "format": "YUV420"},
            controls={"FrameRate": float(self.fps)},
        )
        self.picam.configure(config)
        self.picam.start()

        actual = tuple(self.picam.camera_configuration()["main"]["size"])
        self.get_logger().info(
            f"csi (libcamera): requested {self.width}x{self.height} "
            f"YUV420@{self.fps} -> got {actual[0]}x{actual[1]}")
        if actual != (self.width, self.height):
            # Same calibration caveat as the usb path.
            self.get_logger().warn(
                "libcamera substituted a different size; intrinsics for the "
                "requested mode will NOT apply to these frames")
            self.width, self.height = actual

    def _read_csi(self):
        """Return a mono8 frame from the CSI camera, or None on failure."""
        try:
            yuv = self.picam.capture_array("main")
        except Exception as exc:  # noqa: BLE001 - surface as a read failure
            if self.read_failures == 0:
                self.get_logger().warn(f"csi capture raised: {exc}")
            return None
        # YUV420 planar: the first `height` rows are the luma (Y) plane. Slice
        # width too in case libcamera padded the stride.
        return np.ascontiguousarray(yuv[:self.height, :self.width])

    # ----------------------------- shared ----------------------------------
    def grab(self):
        mono = self._read_usb() if self.source == "usb" else self._read_csi()
        if mono is None:
            self.read_failures += 1
            if self.read_failures in (1, 10, 100) or self.read_failures % 500 == 0:
                self.get_logger().warn(
                    f"read failed ({self.read_failures} total)")
            return
        self.read_failures = 0

        now = self.get_clock().now()
        if self.min_interval > 0.0:
            t = now.nanoseconds * 1e-9
            if t - self.last_publish < self.min_interval:
                return
            self.last_publish = t

        msg = self.bridge.cv2_to_imgmsg(mono, encoding="mono8")
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        if self.picam is not None:
            self.picam.stop()
            self.picam.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExtCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
