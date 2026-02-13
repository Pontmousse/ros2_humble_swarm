#!/usr/bin/env python3
import rclpy
import time
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import Imu
from marvelmind_ros2_msgs.msg import HedgePositionAddressed
from swarm_interfaces.msg import HedgeImuAddressed, CoordXY, Neighbours


class HedgehogNode(Node): 
    def __init__(self):
        super().__init__("hedgehog_obs")

        # Parameters
        self.declare_parameter("robot_name", "RM1")
        self.declare_parameter("beacon_address", 1)
        self.declare_parameter("init_period", 1.0)
        self.declare_parameter("timer_frequency", 0.01)
        self.robot_name = self.get_parameter("robot_name").value
        self.beacon_address = self.get_parameter("beacon_address").value
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

        # Logging
        self.get_logger().info(f"Hedgehog observation node has been started.")
        self.get_logger().info(f"Received robot name: {self.robot_name}")
        self.get_logger().info(f"Received beacon address: {self.beacon_address}\n")
        

        # Gps subscription
        self.pos_sub = self.create_subscription(HedgePositionAddressed, "/mm_all_pos", self.pos_callback, self.qos_profile)
        self.pos_pub = self.create_publisher(HedgePositionAddressed, "mm_pos_unf" ,self.qos_profile)
        self.get_logger().info("Publishing indoor GPS data.")
        
        # IMU calibration
        self.calibrating = True
        self.calib_duration = self.get_parameter("init_period").value
        self.calib_start_time = self.get_clock().now()
        self.accel_buffer = []
        self.gyro_buffer = []
        self.bias_accel = np.zeros(3)
        self.bias_gyro = np.zeros(3)
        self.get_logger().info("Calibrating IMU bias... Please keep the device stationary.")

        # IMU subscription
        self.imu_sub = self.create_subscription(HedgeImuAddressed, "/mm_all_imu", self.imu_callback, self.qos_profile)
        self.imu_pub = self.create_publisher(Imu, "mm_imu" ,self.qos_profile)
        self.get_logger().info("Publishing IMU data.")

        # Publishing timer
        self.pos = HedgePositionAddressed()
        self.imu = Imu()
        self.pub_timer_callback = self.create_timer(self.timer_frequency, self.pub_timer_callback)


        # For Neighbours detection
        self.neighbours = [] # list of neighbouring agent's positions
        self.neighbours_addresses = [] # list of neighbouring agent's ids
        self.neighbours_time_stamps = [] # list of neighbouring agent's observation time stamps

        # publisher and buffer manager
        self.nbh_pub = self.create_publisher(Neighbours, "nbh_pos" ,self.qos_profile)
        self.neighbours_timer = self.create_timer(self.timer_frequency, self.neighbours_buffer_callback)


    def imu_callback(self, msg: HedgeImuAddressed):
        if msg.address != self.beacon_address:
            return

        now = self.get_clock().now()
        elapsed = (now - self.calib_start_time).nanoseconds / 1e9

        # === Calibration Phase ===
        if self.calibrating:
            accel = np.array([
                msg.imu.linear_acceleration.x,
                msg.imu.linear_acceleration.y,
                msg.imu.linear_acceleration.z
            ])
            gyro = np.array([
                msg.imu.angular_velocity.x,
                msg.imu.angular_velocity.y,
                msg.imu.angular_velocity.z
            ])

            self.accel_buffer.append(accel)
            self.gyro_buffer.append(gyro)

            if elapsed >= self.calib_duration:
                self.bias_accel = np.mean(self.accel_buffer, axis=0)
                self.bias_gyro = np.mean(self.gyro_buffer, axis=0)
                self.calibrating = False

                self.get_logger().info("IMU Calibration complete.")
                self.get_logger().info(f"Accel bias: {self.bias_accel}")
                self.get_logger().info(f"Gyro bias: {self.bias_gyro}")
            return

        # === Post-Calibration: Apply bias correction and unit conversion ===
        accel = np.array([
            msg.imu.linear_acceleration.x,
            msg.imu.linear_acceleration.y,
            msg.imu.linear_acceleration.z
        ]) - self.bias_accel

        gyro = np.array([
            msg.imu.angular_velocity.x,
            msg.imu.angular_velocity.y,
            msg.imu.angular_velocity.z
        ]) - self.bias_gyro


        imu_msg = Imu()
        # Apply conversions (mg to m/s^2 and dps to rad/s)
        imu_msg.linear_acceleration.x = accel[0] * 9.80665 / 1000
        imu_msg.linear_acceleration.y = accel[1] * 9.80665 / 1000
        imu_msg.linear_acceleration.z = (accel[2] + 1000) * 9.80665 / 1000  # add gravity

        imu_msg.angular_velocity.x = gyro[0] * 0.0175 * np.pi / 180
        imu_msg.angular_velocity.y = gyro[1] * 0.0175 * np.pi / 180
        imu_msg.angular_velocity.z = gyro[2] * 0.0175 * np.pi / 180

        # timestamp and publish
        imu_msg.header.stamp = msg.imu.header.stamp
        self.imu = imu_msg

    def pub_timer_callback(self):
        # publish IMU data
        self.imu_pub.publish(self.imu)

        # publish position data
        self.pos_pub.publish(self.pos)
        # publish neighbours data

    def pos_callback(self, msg: HedgePositionAddressed):
        if msg.address == self.beacon_address:
            self.pos = msg
        
        ################################################################################################
        # Neighbours
        ################################################################################################
        
        else:
            # extract neighbour address
            adr = msg.address
            pos = [msg.x_m, msg.y_m, msg.z_m]
            time_stamp = self.get_clock().now()

            # Check if neighbour is already in buffer, if yes update its position/timestamp
            if adr in self.neighbours_addresses:
                idx = self.neighbours_addresses.index(adr)

                # update neighbour position and timestamp
                self.neighbours[idx] = pos
                self.neighbours_time_stamps[idx] = time_stamp
            
            # if not, add neighbour position/timestamp/address to buffer.
            else:
                self.neighbours_addresses.append(adr)
                self.neighbours.append(pos)
                self.neighbours_time_stamps.append(time_stamp)

    def neighbours_buffer_callback(self):
        now = self.get_clock().now()

        for timestamp in self.neighbours_time_stamps:
            if now - timestamp > Duration(seconds=2.0): # seconds of buffer
                idx = self.neighbours_time_stamps.index(timestamp)
                # delete unseen neighboor
                del self.neighbours[idx]
                del self.neighbours_addresses[idx]
                del self.neighbours_time_stamps[idx]


        msg = Neighbours()
        for i in range(len(self.neighbours)):
            pos = CoordXY()
            pos.x = self.neighbours[i][0]
            pos.y = self.neighbours[i][1]
            msg.neighbours.append(pos)
            
        self.nbh_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HedgehogNode() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
