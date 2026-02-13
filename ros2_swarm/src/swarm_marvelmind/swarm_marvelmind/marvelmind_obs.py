#!/usr/bin/env python3
import rclpy
import time
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from swarm_marvelmind.marvelmind import MarvelmindHedge

from sensor_msgs.msg import Imu
from marvelmind_ros2_msgs.msg import HedgePositionAddressed
from swarm_interfaces.msg import HedgeImuAddressed

class MarvelmindNode(Node): 
    def __init__(self):
        super().__init__("marvelmind_obs")

        self.declare_parameter("robot_names", ["RM1"])
        self.declare_parameter("beacon_addresses", [1])
        self.declare_parameter("port_address", "/dev/ttyACM0")

        self.robot_names = self.get_parameter("robot_names").get_parameter_value().string_array_value
        self.beacon_addresses = self.get_parameter("beacon_addresses").get_parameter_value().integer_array_value
        self.N = len(self.robot_names) # number of beacons/robots in the swarm

        self.get_logger().info(f"Received robot_names: {self.robot_names}")
        self.get_logger().info(f"Received beacon_addresses: {self.beacon_addresses}")
        self.get_logger().info(f"There are {self.N} beacons/robots in the swarm")

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

        #####################

        self.tty = self.get_parameter("port_address").get_parameter_value().string_value
        self.hedge = MarvelmindHedge(tty=self.tty, adr=None, debug=False)
        self.hedge.start()  # Start listening to Marvelmind data
        self.get_logger().info("Marvelmind observation node has been started.")

        #####################

        self.time_period = 0.001

        self.get_logger().info("Publishing indoor GPS data.")
        self.pos_pub = self.create_publisher(HedgePositionAddressed, "/mm_all_pos" ,self.qos_profile)
        self.pos_timer = self.create_timer(self.time_period, self.publish_pos_callback)

        self.get_logger().info("Publishing IMU data.")
        self.imu_pub = self.create_publisher(HedgeImuAddressed, "/mm_all_imu" ,self.qos_profile)
        self.imu_timer = self.create_timer(self.time_period, self.publish_imu_callback)

    def publish_imu_callback(self):
        msg = HedgeImuAddressed()
        imu_msg = Imu()
        if self.hedge.rawImuUpdated:
            # self.get_logger().info("Raw IMU data updated.")
            self.hedge.rawImuUpdated = False # set back to False to not keep entering the condition and triggering the callback content - comes back only if rawImuUpdated is True again
            temp = self.hedge.raw_imu()

            # Debug
            # self.get_logger().info(f"imu: {temp}\n") 
            # self.hedge.print_raw_imu()

            # Remove bias
            accel = np.array([temp[1], temp[2], temp[3]]) 
            gyro = np.array([temp[4], temp[5], temp[6]])

            imu_msg.linear_acceleration.x = float(accel[0])
            imu_msg.linear_acceleration.y = float(accel[1])
            imu_msg.linear_acceleration.z = float(accel[2])
            
            imu_msg.angular_velocity.x = float(gyro[0]) 
            imu_msg.angular_velocity.y = float(gyro[1])
            imu_msg.angular_velocity.z = float(gyro[2])

            # timestamp
            imu_msg.header.stamp = self.get_clock().now().to_msg() 


            # Fill in main message
            msg.address = temp[0]
            msg.imu = imu_msg
            self.imu_pub.publish(msg)


    def publish_pos_callback(self):
        msg_pos = HedgePositionAddressed()
        if self.hedge.positionUpdated:
            # self.get_logger().info("Position data updated.")
            self.hedge.positionUpdated = False # set back to False to not keep entering the condition and triggering the callback content - comes back only if rawImuUpdated is True again
            temp = self.hedge.position()

            # Debug
            # self.get_logger().info(f"pos: {temp}\n") 
            # self.hedge.print_position()

            # populate position values in meters
            msg_pos.x_m = temp[1]
            msg_pos.y_m = temp[2]
            msg_pos.z_m = temp[3]

            # not used but just populate
            msg_pos.timestamp_ms = temp[5]
            msg_pos.address = temp[0]

            self.pos_pub.publish(msg_pos)


def main(args=None):
    rclpy.init(args=args)
    node = MarvelmindNode() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
