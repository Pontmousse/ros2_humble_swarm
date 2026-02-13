import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math, numpy as np

# Custom messages
from swarm_interfaces.msg import StatePos as State 
from swarm_interfaces.msg import CoordXY, Markers, Neighbours


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
        self.qos_profile_reliable = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.qos_profile_best_effort = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # # # # # # # # # # # # # # # # # # # # # # # # # # #

        # Control states
        self.pose = np.array([None, None, None])  # x, y, theta
        self.landmarks = [] # list of landmark coordinates in global reference frame
        self.target = np.array([None, None])      # target position
        self.neighbours = [] # list of neighbouring agent's positions


        # Control gains
        self.Rflk = 0.4
        self.Rant = 0.4
        self.flk_gain = 0.5
        self.antflk_gain = 0.1
        self.eps = 1e-4
        

        # velocity commands
        self.vel1 = np.array([0, 0])
        self.vel2 = np.array([0, 0])
        self.step = 1.0

        # Subscribers
        self.create_subscription(State, 'pose', self.pose_callback, self.qos_profile_best_effort)
        self.create_subscription(Markers, 'landmarks', self.landmarks_callback, self.qos_profile_best_effort)
        self.create_subscription(Neighbours, 'nbh_pos', self.neighbours_callback, self.qos_profile_best_effort)
        

        # Publishers
        self.pub = self.create_publisher(CoordXY, 'target_position', self.qos_profile_reliable)
        self.pub_timer = self.create_timer(self.timer_frequency, self.publish_target_position)

    def pose_callback(self, msg: State):
        self.pose = np.array([msg.x, msg.y, msg.theta])

    def landmarks_callback(self, msg: Markers):
        # Check if pose is initialized
        if None in self.pose:
            self.get_logger().warn("Pose not initialized yet.")
            self.vel1 = np.array([0, 0])
            return

        # Check if landmarks exist
        if len(msg.markers) == 0:
            # self.get_logger().warn("Received empty landmarks.")
            self.vel1 = np.array([0, 0])
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
        self.vel1 = self.flocking_potential(self.landmarks)   


    def flocking_potential(self, landmarks):
        Fx, Fy = 0.0, 0.0
        x, y = self.pose[0], self.pose[1]
        
        for i in range(len(landmarks)):
            xp, yp = landmarks[i, 0], landmarks[i, 1]
            dx = xp - x
            dy = yp - y
            distance = np.hypot(dx, dy) + self.eps

            # Spring-like attractive/repulsive force
            force_magnitude = self.flk_gain * (distance - self.Rflk)  # Attract if > Rflk, repel if < Rflk

            Fx += force_magnitude * (dx / distance)
            Fy += force_magnitude * (dy / distance)

        return np.array([Fx, Fy])

    
    ######################################################################################################

    def neighbours_callback(self, msg: Neighbours):
        # Check if pose is initialized
        if None in self.pose:
            self.get_logger().warn("Pose not initialized yet.")
            self.vel1 = np.array([0, 0])
            return
        
        # Check if neighbors exist
        if len(msg.neighbours) == 0:
            # self.get_logger().warn("Received empty neighbours.")
            self.vel2 = np.array([0, 0])
            return

        # Extract landmarks
        self.neighbours = make_array(msg.neighbours) # in meters

        # Calculate force direction and amplitude
        self.vel2 = self.antiflocking_potential(self.neighbours)
        # self.get_logger().info(f"self.neighbours: {self.neighbours}") # PRINT LOG DEBUG
        # self.get_logger().info(f"Computed vel2 (anti-flocking): {self.vel2}") # PRINT LOG DEBUG

    def antiflocking_potential(self, neighbours):
        Fx, Fy = 0.0, 0.0
        x, y = self.pose[0], self.pose[1]

        # self.get_logger().info(f"\nx = self.pose[0]: {self.pose[0]}") # PRINT LOG DEBUG
        # self.get_logger().info(f"x = self.pose[1]: {self.pose[1]}\n") # PRINT LOG DEBUG
        
        for i in range(len(neighbours)):
            xp, yp = neighbours[i, 0], neighbours[i, 1]
            dx = x - xp
            dy = y - yp
            distance = np.hypot(dx, dy) + self.eps

            # self.get_logger().info(f"distance: {distance}") # PRINT LOG DEBUG

            if distance < self.Rant:  # Only apply force within influence radius
                factor = self.antflk_gain * (1.0 / distance - 1.0 / self.Rant) / (distance ** 2)
                Fx += factor * (dx / distance)
                Fy += factor * (dy / distance)

        return np.array([Fx, Fy])


    ######################################################################################################

    def publish_target_position(self):
        # Wait for initialization before triggering the controller
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        if elapsed <= 2 * self.init_duration:
            return
        ############################################################

        if None in self.pose:
            self.get_logger().warn("Pose not initialized yet, skipping publish.")
            return

        # Calculate next waypoint position target
        self.target = self.pose[:2] + self.step * (self.vel1 + self.vel2)
        # self.get_logger().warn(f"switched position target to: {self.target}\n") 

        msg = CoordXY()
        msg.x = self.target[0]
        msg.y = self.target[1]
        self.pub.publish(msg)

def make_array(list_coord):
    # list_coord can be Landmarks or Neighbours => convert to Numpy array n both cases

    M = len(list_coord)
    arr = np.zeros((M, 2))
    for i in range(M):
        arr[i, 0] = list_coord[i].x
        arr[i, 1] = list_coord[i].y
    return arr



def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()