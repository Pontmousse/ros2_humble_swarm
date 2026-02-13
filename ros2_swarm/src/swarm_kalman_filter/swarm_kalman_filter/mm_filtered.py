import rclpy
from rclpy.node import Node
from marvelmind_ros2_msgs.msg import HedgePositionAddressed
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PositionFilterNode(Node):
    def __init__(self):
        super().__init__('position_filter')

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

        # Filter coefficient: alpha in [0.0, 1.0], where smaller = heavier smoothing
        self.declare_parameter('alpha', 0.3)
        self.alpha: float = self.get_parameter('alpha').get_parameter_value().double_value

        # Place to store last filtered values
        self._filtered_msg: HedgePositionAddressed | None = None

        # Subscriber (un-filtered data)
        self.subscription = self.create_subscription(
            HedgePositionAddressed,
            'mm_pos_unf',
            self.mm_pos_callback,
            self.qos_profile,
        )

        # Publisher (smoothed data)
        self.publisher = self.create_publisher(
            HedgePositionAddressed,
            'mm_pos',
            self.qos_profile,
        )

        self.get_logger().info(f'Low-pass filter node started (alpha={self.alpha})')

    def mm_pos_callback(self, msg: HedgePositionAddressed):
        # On first message, initialize the filtered state
        if self._filtered_msg is None:
            self._filtered_msg = HedgePositionAddressed()
            self._filtered_msg.address = msg.address
            self._filtered_msg.timestamp_ms = msg.timestamp_ms
            self._filtered_msg.x_m = msg.x_m
            self._filtered_msg.y_m = msg.y_m
            self._filtered_msg.z_m = msg.z_m
        else:
            # Always carry over any non-position fields
            self._filtered_msg.address = msg.address
            self._filtered_msg.timestamp_ms = msg.timestamp_ms

            # Exponential moving average: y[n] = α x[n] + (1−α) y[n−1]
            α = self.alpha
            self._filtered_msg.x_m = α * msg.x_m + (1 - α) * self._filtered_msg.x_m
            self._filtered_msg.y_m = α * msg.y_m + (1 - α) * self._filtered_msg.y_m
            self._filtered_msg.z_m = α * msg.z_m + (1 - α) * self._filtered_msg.z_m

        # Publish the filtered message
        self.publisher.publish(self._filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PositionFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
