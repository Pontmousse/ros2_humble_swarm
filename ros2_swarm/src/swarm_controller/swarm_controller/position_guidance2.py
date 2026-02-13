import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import math, numpy as np

# Custom messages
from swarm_interfaces.msg import StatePos as State 
from swarm_interfaces.msg import CoordXY, Markers


class GuidanceNode(Node):
    def __init__(self):
        super().__init__('target_position_controller')
        self.get_logger().info("Target position controller has been started.")

        # For initial pose initialization using GPS / Odometry readings
        self.declare_parameter("init_period", 1.0)
        self.init_duration = self.get_parameter("init_period").value
        self.init_start_time = self.get_clock().now()

        # For timer
        self.declare_parameter("timer_frequency", 0.01)
        self.timer_frequency = self.get_parameter("timer_frequency").value

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

        # Control states
        self.pose = np.array([None, None, None])  # x, y, theta
        self.landmarks = [] # list of landmark coordinates in global reference frame
        self.target = np.array([None, None])      # target position

        # Control gains
        self.Rflk = 0.4
        self.gain = 0.2
        self.eps = 1e-4
        self.step = 1.0

        # Subscribers
        self.create_subscription(State, 'pose', self.pose_callback, self.qos_profile)
        self.create_subscription(Markers, 'landmarks', self.landmarks_callback, self.qos_profile)

        # Publishers
        self.pub = self.create_publisher(CoordXY, 'target_position', self.qos_profile)
        self.pub_timer = self.create_timer(self.timer_frequency, self.publish_target_position)

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

        self.landmarks = self.pose[:2] + np.matmul(rot_matrix, landmarks.T).T

        # Calculate force direction and amplitude
        self.vel = self.attractive_potential(self.landmarks)

        # Calculate next waypoint position target
        self.target = self.pose[:2] + self.step * self.vel
        # self.get_logger().warn(f"switched position target to: {self.target}\n")


    def attractive_potential(self, landmarks):
        Fx, Fy = 0.0, 0.0
        for i in range(len(landmarks)):
            xp = landmarks[i,0]
            yp = landmarks[i,1]

            x = self.pose[0]
            y = self.pose[1]
            ##############################

            rp = np.array([xp, yp])
            r = np.array([x, y])
            Nr = np.linalg.norm(r - rp)

            # X direction
            Xd = (x**2)/(x + self.eps) - (xp**2)/(xp + self.eps)
            numer_x = abs(x - xp) * (self.Rflk - Nr) * (x - xp + Xd)
            denom_x = Nr * np.sqrt((x - xp) * Xd + self.eps)
            Fx += (self.gain / 2.0) * (numer_x / (denom_x + self.eps))

            # Y direction
            Yd = (y**2)/(y + self.eps) - (yp**2)/(yp + self.eps)
            numer_y = abs(y - yp) * (self.Rflk - Nr) * (y - yp + Yd)
            denom_y = Nr * np.sqrt((y - yp) * Yd + self.eps)
            Fy += (self.gain / 2.0) * (numer_y / (denom_y + self.eps))
            
        force = np.array([Fx, Fy])
        return force


    def publish_target_position(self):
        # Wait for initialization before triggering the controller
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        if elapsed <= 2 * self.init_duration:
            return
        ############################################################
        
        if None in self.target:
            self.get_logger().warn("Target position not initialized yet.")
            return

        msg = CoordXY()
        msg.x = self.target[0]
        msg.y = self.target[1]
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