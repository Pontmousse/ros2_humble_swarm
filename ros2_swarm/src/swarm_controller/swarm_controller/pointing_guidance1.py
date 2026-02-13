import rclpy
from rclpy.node import Node


import random, numpy as np

# Custom messages
from std_msgs.msg import Float64

class GuidanceNode(Node):
    def __init__(self):
        super().__init__('target_orientation_controller')
        self.get_logger().info("Target orientation controller has been started.")

        # For initial pose initialization using GPS / Odometry readings
        self.declare_parameter("init_period", 1.0)
        self.init_duration = self.get_parameter("init_period").value
        self.init_start_time = self.get_clock().now()

        self.target = 0.0

        # Publishers
        self.pub = self.create_publisher(Float64, 'target_orientation', 10)
        self.pub_timer = self.create_timer(5, self.publish_target_orientation)

    def publish_target_orientation(self):
        # Wait for initialization before triggering the controller
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        if elapsed <= 2 * self.init_duration:
            return
        

        theta = random.uniform(-90.0,90.0)
        self.target += theta
        self.target = (self.target + 180) % 360 - 180

        # self.target = 90.0
        self.get_logger().warn(f"switched orientation target to: {self.target}\n")

        msg = Float64()
        msg.data = self.target
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
