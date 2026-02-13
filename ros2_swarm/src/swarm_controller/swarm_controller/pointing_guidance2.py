import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import math, numpy as np

# Custom messages
from swarm_interfaces.msg import StatePos as State 
from swarm_interfaces.msg import Markers
from std_msgs.msg import Float64

class GuidanceNode(Node):
    def __init__(self):
        super().__init__('landmark_tracking_controller')
        self.get_logger().info("Landmark tracking controller has been started.")

        # For initial pose initialization using GPS / Odometry readings
        self.declare_parameter("init_period", 1.0)
        self.init_duration = self.get_parameter("init_period").value
        self.init_start_time = self.get_clock().now()

        # For timer
        self.declare_parameter("timer_frequency", 0.01)
        self.timer_frequency = self.get_parameter("timer_frequency").value
        self.get_logger().info(f"Timer frequency: {self.timer_frequency} seconds")

        # # # # # # # # # # # # # # # # # # # # # # # # # # # 
        # For QoS
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'RELIABLE')
        self.declare_parameter('qos_history', 'KEEP_LAST')

        qos_depth = self.get_parameter('qos_depth').value
        qos_reliability = self.get_parameter('qos_reliability').value
        qos_history = self.get_parameter('qos_history').value

        # Validate QoS settings       
        valid_reliabilities = ['RELIABLE', 'BEST_EFFORT']
        valid_histories = ['KEEP_LAST', 'KEEP_ALL']

        if qos_reliability not in valid_reliabilities:
            raise ValueError(f"Invalid qos_reliability: {qos_reliability}")
        if qos_history not in valid_histories:
            raise ValueError(f"Invalid qos_history: {qos_history}")

        # QoS Settings 
        self.qos_profile = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE if qos_reliability == 'RELIABLE' else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST if qos_history == 'KEEP_LAST' else HistoryPolicy.KEEP_ALL
        )
        # # # # # # # # # # # # # # # # # # # # # # # # # # # 

        # Control state
        self.pose = np.array([None, None, None])  # x, y, theta
        self.lc = np.array([None, None])          # landmark centroid
        self.target_lcd = None                    # direction (deg)
        
        # Subscribers
        self.create_subscription(State, 'pose', self.pose_callback, self.qos_profile)
        self.create_subscription(Markers, 'landmarks', self.landmarks_callback, self.qos_profile)

        # Publishers
        self.pub = self.create_publisher(Float64, 'target_orientation', self.qos_profile)
        self.pub_timer = self.create_timer(self.timer_frequency, self.publish_target_orientation)


    def pose_callback(self, msg: State):
        self.pose = np.array([msg.x, msg.y, msg.theta])



    def landmarks_callback(self, msg: Markers):
        # Check if pose is initialized
        if None in self.pose:
            self.get_logger().warn("Pose not initialized yet.")
            return

        # Check if landmarks exist
        if len(msg.markers) == 0:
            # self.get_logger().warn("Received empty landmarks.")
            return

        # Extract landmarks
        landmarks = make_array(msg.markers) / 100.0  # Convert cm to m

        # Transform to global frame from local robot frame
        theta_rad = math.radians(self.pose[2])
        rot_matrix = np.array([
            [math.cos(theta_rad), -math.sin(theta_rad)],
            [math.sin(theta_rad),  math.cos(theta_rad)]
        ])
        # self.get_logger().warn(f"rotation: {rot_matrix}\n")
        # self.get_logger().warn(f"landmarks: {landmarks.T}\n")
        landmarks_global = self.pose[:2] + np.matmul(rot_matrix, landmarks.T).T

        # Calculate landmark centroid in global frame
        self.lc = np.mean(landmarks_global, axis=0)

        # Compute direction to centroid
        dir_vec = self.lc - self.pose[:2]
        if np.linalg.norm(dir_vec) == 0:
            self.get_logger().warn("Zero direction vector to centroid.")
            return

        dir_vec /= np.linalg.norm(dir_vec)
        self.target_lcd = math.degrees(math.atan2(dir_vec[1], dir_vec[0]))  # in degrees

        # self.get_logger().warn(f"switched orientation target to: {self.target_lcd}\n")



    def publish_target_orientation(self):
        # Wait for initialization before triggering the controller
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        if elapsed <= 2 * self.init_duration:
            return

        if self.target_lcd == None:
            return

        msg = Float64()
        msg.data = self.target_lcd
        self.pub.publish(msg)


def make_array(landmarks):
    M = len(landmarks)
    arr = np.zeros((M, 2))
    for i in range(M):
        arr[i, 0] = landmarks[i].x
        arr[i, 1] = landmarks[i].y
    return arr


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
