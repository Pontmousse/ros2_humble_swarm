import rclpy
from rclpy.node import Node
import random, numpy as np

# Custom messages
from swarm_interfaces.msg import CoordXY


class GuidanceNode(Node):
    def __init__(self):
        super().__init__('target_position_controller')
        self.get_logger().info("Target position controller has been started.")

        # For initial pose initialization using GPS / Odometry readings
        self.declare_parameter("init_period", 1.0)
        self.init_duration = self.get_parameter("init_period").value

        self.target = np.array([2.0, 1.0])
        self.init_start_time = self.get_clock().now()

        # Publishers
        self.pub = self.create_publisher(CoordXY, 'target_position', 10)
        self.pub_timer = self.create_timer(5, self.publish_target_position)

    def publish_target_position(self):
        # Wait for initialization before triggering the controller
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        if elapsed <= 2 * self.init_duration:
            return
        ############################################################

        x = random.uniform(1.5,3.5)
        y = random.uniform(1.0,2.5)

        self.target = np.array([1.3, 5.7]) # custom position
        # self.target = np.array([3.5, 1.9]) # Keele 0151 chair position

        # self.target = np.array([x, y]) # random position
        
        self.get_logger().warn(f"switched position target to: {self.target}\n")

        msg = CoordXY()
        msg.x = self.target[0]
        msg.y = self.target[1]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()