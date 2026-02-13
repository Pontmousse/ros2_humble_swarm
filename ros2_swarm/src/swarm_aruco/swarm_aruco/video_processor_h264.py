import rclpy
from rclpy.node import Node

import av
import cv2
import numpy as np

from std_msgs.msg import Header
from robomaster_msgs.msg import H264Packet  # Replace with your actual package name

class H264VideoProcessor(Node):
    def __init__(self, display=False):
        super().__init__('video_processor')

        self.display = display
        self.decoder = av.codec.CodecContext.create('h264', 'r')

        self.subscription = self.create_subscription(
            H264Packet,
            'camera/image_h264', 
            self.h264_callback,
            10
        )

        self.get_logger().warn('H264 video processor initialized. Display: {}'.format(self.display))

    def h264_callback(self, msg):
        try:
            packets = self.decoder.parse(msg.data)
            for packet in packets:
                frames = self.decoder.decode(packet)
                for frame in frames:
                    img = frame.to_ndarray(format='bgr24')
                    
                    # save in main ROS2 workspace directory
                    cv2.imwrite('h264_test_frame.jpg', img)

                    # Optional: Show it (if GUI is working)
                    if self.display:
                        cv2.imshow("H264 Image Stream", img)
                        cv2.waitKey(1)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        rclpy.shutdown()

                    self.get_logger().info(f"I am here and the image shape is: {img.shape}") # Debug
        except av.OSError as e:
            self.get_logger().warn(f"H264 decode error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = H264VideoProcessor(display=False) # Set to True if you want to enable display, but might not work
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
