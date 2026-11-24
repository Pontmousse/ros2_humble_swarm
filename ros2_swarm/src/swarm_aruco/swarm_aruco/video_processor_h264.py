"""Decode the RoboMaster H.264 stream once and republish mono8 on `camera/image_color`.

Same output contract as ext_camera, so both aruco detectors (use_h264: False)
share one decode instead of each running their own.
"""
import time

import av
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from robomaster_msgs.msg import H264Packet
from sensor_msgs.msg import Image


class H264VideoProcessor(Node):
    def __init__(self):
        super().__init__('video_processor')
        # Every packet is decoded (H.264 frames reference each other); only the
        # gray conversion + publish is capped. 0 disables the cap.
        self.declare_parameter('publish_max_hz', 15.0)
        hz = float(self.get_parameter('publish_max_hz').value)
        self.min_interval = 1.0 / hz if hz > 0 else 0.0
        self.last_publish = 0.0
        # Low-rate copy of the decoded frame on camera/image_snapshot for the rosbag
        # (a lone H.264 packet is undecodable without its keyframe). 0 disables.
        self.declare_parameter('snapshot_period', 10.0)
        self.snapshot_period = float(self.get_parameter('snapshot_period').value)
        self.last_snapshot = 0.0
        self.snap_pub = self.create_publisher(Image, 'camera/image_snapshot', 1)
        self.decoder = av.codec.CodecContext.create('h264', 'r')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'camera/image_color', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(H264Packet, 'camera/image_h264', self.h264_callback, 10)
        self.get_logger().info(f'H264 decoder up, publishing mono8 at <= {hz} Hz')

    def h264_callback(self, msg):
        try:
            for packet in self.decoder.parse(bytes(msg.data)):  # data is array.array('B')
                try:
                    frames = self.decoder.decode(packet)
                except av.error.InvalidDataError as e:
                    self.get_logger().warn(f'Invalid data during decode: {e}', throttle_duration_sec=5.0)
                    continue
                for frame in frames:
                    t = time.monotonic()
                    if t - self.last_publish < self.min_interval:
                        continue
                    self.last_publish = t
                    out = self.bridge.cv2_to_imgmsg(frame.to_ndarray(format='gray'), encoding='mono8')
                    out.header = msg.header
                    self.pub.publish(out)
                    if self.snapshot_period > 0.0 and t - self.last_snapshot >= self.snapshot_period:
                        self.last_snapshot = t
                        self.snap_pub.publish(out)
        except av.FFmpegError as e:
            self.get_logger().warn(f'H264 decode error: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = H264VideoProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
