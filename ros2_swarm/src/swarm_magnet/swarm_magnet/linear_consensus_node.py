#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from swarm_interfaces.msg import CoordXY, Neighbours


class LinearConsensusNode(Node):
    def __init__(self):
        super().__init__("linear_consensus_node")

        # Parameters
        self.declare_parameter("robot_name", "agent_1")
        self.declare_parameter("initial_position", [0.0, 0.0])
        self.declare_parameter("consensus_rate", 1.0)  # gain per second
        self.declare_parameter("timer_frequency", 1.0)  # Hz

        self.robot_name = self.get_parameter("robot_name").value
        self.position = np.array(self.get_parameter("initial_position").value, dtype=float)
        consensus_rate = self.get_parameter("consensus_rate").value
        self.timer_frequency = self.get_parameter("timer_frequency").value

        # Compute time step
        self.dt = self.timer_frequency
        self.alpha = consensus_rate * self.dt  # scaled gain per update step

        self.get_logger().info(f"[{self.robot_name}] Linear consensus node started.")
        self.get_logger().info(f"Initial position: {self.position.tolist()}")
        self.get_logger().info(f"Consensus gain per second: {consensus_rate}")
        self.get_logger().info(f"Consensus update step: {self.alpha:.4f} (with Δt = {self.dt:.2f}s)")

        # QoS settings
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publishers & Subscribers
        self.pos_pub = self.create_publisher(CoordXY, "consensus_position", qos_profile)
        self.nbh_sub = self.create_subscription(Neighbours, "nbh_pos", self.neighbours_callback, qos_profile)

        # Timer for consensus update
        self.timer = self.create_timer(self.timer_frequency, self.consensus_update)

        # Store latest neighbor data
        self.neighbours = []

    def neighbours_callback(self, msg: Neighbours):
        self.neighbours = [(n.x, n.y) for n in msg.neighbours]
        self.get_logger().debug(f"[{self.robot_name}] Received {len(self.neighbours)} neighbours.")

    def consensus_update(self):
        if not self.neighbours:
            return

        own_pos = self.position
        neighbor_positions = np.array(self.neighbours)
        mean_neighbor = np.mean(neighbor_positions, axis=0)

        delta = self.alpha * (mean_neighbor - own_pos)
        self.position += delta

        msg = CoordXY()
        msg.x = float(self.position[0])
        msg.y = float(self.position[1])
        self.pos_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LinearConsensusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
