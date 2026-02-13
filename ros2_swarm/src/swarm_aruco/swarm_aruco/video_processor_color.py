import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ColorVideoProcessor(Node):
    def __init__(self, display=False):
        super().__init__('color_image_processor')

        self.display = display
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera/image_color', 
            self.image_callback,
            10
        )

        self.get_logger().info('Color video processor initialized. Display: {}'.format(self.display))

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Save one test image in ros2 workspace
            cv2.imwrite('color_test_frame.jpg', img)

            # Optional: Show it (if GUI is working)
            if self.display:
                cv2.imshow("Color Image Stream", img)
                cv2.waitKey(1)

            self.get_logger().warn(f"Received color image with shape: {img.shape}")

        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ColorVideoProcessor(display=False)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
