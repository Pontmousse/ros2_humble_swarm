import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from filterpy.kalman import KalmanFilter
import numpy as np

from marvelmind_ros2_msgs.msg import HedgePositionAddressed
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from swarm_interfaces.msg import StateFull as State # Custom message

def degrees(angle):
    # The angle in the Kalman filter state is in radians, this is just when publishing, we publish in degrees
    # takes angle in radians (radian per second)
    # convert it to degree between -180 and 180 degrees

    angle = angle / np.pi * 180 # convert to degree
    angle = (angle + 180) % 360 - 180 # wrap angle

    return angle

def low_pass_filter(new_value, prev_value, alpha):
    """
    Apply a low-pass filter to smooth the new value.
    
    Args:
        new_value: Current value to filter.
        prev_value: Previous filtered value.
        alpha: Smoothing factor (0 < alpha < 1).
        
    Returns:
        Smoothed value.
    """
    return alpha * new_value + (1 - alpha) * prev_value

###################################################################################################################33

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter')
        self.get_logger().info("Kalman filter has been started.")

        self.declare_parameter("init_period", 1.0)

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

        # Kalman Filter for [x, y, theta, vx, vy, omega]
        self.kf = KalmanFilter(dim_x=6, dim_z=5)
        self.init_kf()

        # For initial state / noise levels initialization using GPS /IMU readings
        self.initializing = True
        self.init_duration = self.get_parameter("init_period").value
        self.init_start_time = self.get_clock().now()
        self.pos_buffer = []
        self.init_pos = np.zeros(2)
        self.get_logger().info("Initializing initial position & noise levels... Please keep the devices stationary.")

        # Subscriptions (GPS, Odometry, IMU + MM_IMU)
        self.create_subscription(HedgePositionAddressed, 'mm_pos', self.indoor_gps_callback, self.qos_profile)
        self.create_subscription(Odometry, 'odom', self.odom_callback, self.qos_profile)
        self.create_subscription(Imu, 'imu', self.imu_callback, self.qos_profile)
        self.create_subscription(Imu, 'mm_imu', self.mm_imu_callback, self.qos_profile)

        # Publisher for estimated state
        self.kf_pub = self.create_publisher(State, 'kf_state', self.qos_profile)

        # Predict timer
        self.declare_parameter("timer_frequency", 0.01)
        self.timer_frequency = self.get_parameter("timer_frequency").value
        self.predict_timer = self.create_timer(self.timer_frequency, self.predict_step) 

        # Internal tracking
        self.alpha = 0.1 # low pass filter coeffecient
        self.latest_accel = np.zeros(3)  # ax, ay, omega
        self.latest_mm_accel = np.zeros(3)  # ax, ay, omega
        self.last_predict_time = None

    def init_kf(self):
        # Initial state: [x, y, theta, vx, vy, omega]
        self.kf.x = np.zeros(6)

        # Set initial orientation
        self.declare_parameter("init_orientation", 0.0) # in degrees
        self.kf.x[2] = self.get_parameter("init_orientation").value / 180 * np.pi

        # Initial uncertainty
        self.kf.P *= 0.1

        # Process noise
        self.kf.Q = np.eye(6) * 0.01

        # Sensor noise
        self.R = np.diag([0.02, 0.02, 0.1, 0.1, 0.05])

    ########################################################################

    def imu_callback(self, msg: Imu):
        current_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.angular_velocity.z
        ])
        self.latest_accel = low_pass_filter(current_accel, self.latest_accel, self.alpha)

    def mm_imu_callback(self, msg: Imu):
        current_mm_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.angular_velocity.z
        ])
        self.latest_mm_accel = low_pass_filter(current_mm_accel, self.latest_mm_accel, self.alpha)

    ########################################################################

    def predict_step(self):
        if self.initializing:
                return
        
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.last_predict_time is None:
            self.last_predict_time = now
            return

        dt = now - self.last_predict_time
        self.last_predict_time = now

        # F: State transition matrix
        F = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1]
        ])

        # B: Control input model for [ax, ay, alpha]
        B = np.array([
            [0.5 * dt**2, 0,           0],
            [0,           0.5 * dt**2, 0],
            [0,           0,           dt],
            [dt,          0,           0],
            [0,           dt,          0],
            [0,           0,           0]
        ])

        accel = (self.latest_accel + self.latest_mm_accel)/2 # average reading of both IMUs
        self.kf.F = F
        self.kf.predict(u=accel, B=B)

    ########################################################################

    def indoor_gps_callback(self, msg: HedgePositionAddressed):
        now = self.get_clock().now()
        elapsed = (now - self.init_start_time).nanoseconds / 1e9

        # === Initialization Phase ===
        if self.initializing:
            pos = np.array([
                msg.x_m,
                msg.y_m
            ])
            self.pos_buffer.append(pos)

            if elapsed >= self.init_duration:
                self.init_pos = np.mean(self.pos_buffer, axis=0)
                self.kf.x[:2] = self.init_pos
                self.initializing = False
                self.get_logger().info("Indoor GPS initialization complete.")
                self.get_logger().info(f"Initial position: {self.init_pos}")
                self.get_logger().info(f"Initial state: {self.kf.x}")
            return
        
        ### ============================ ### ============================ ###

        # GPS measures [x, y]
        H_gps = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]
        ])
        z = np.array([msg.x_m, msg.y_m, 0, 0, 0])

        self.kf.update(z, H=H_gps, R=self.R)
        self.publish_state()

    ########################################################################

    def odom_callback(self, msg: Odometry):
        if self.initializing:
                return
        
        # Odometry measures [vx, vy, omega]
        H_odom = np.array([
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        z = np.array([0, 0, msg.twist.twist.linear.x, msg.twist.twist.linear.x, msg.twist.twist.angular.z])

        self.kf.update(z, H=H_odom, R=self.R)
        self.publish_state()

    ########################################################################

    def publish_state(self):
        msg = State()

        msg.x = self.kf.x[0]
        msg.y = self.kf.x[1]
        theta = self.kf.x[2] 
        msg.theta = degrees(theta)

        msg.vx = self.kf.x[3]
        msg.vy = self.kf.x[4]
        omega = self.kf.x[5]
        msg.omega = degrees(omega)

        self.kf_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
