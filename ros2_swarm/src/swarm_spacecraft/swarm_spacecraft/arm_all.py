# ROS 2 Python Node to Arm the Virtual Spacecraft Simulation on selected robots (default: RM1 to RM10)

import argparse
from typing import Any

import rclpy
import rclpy.utilities
from rclpy.node import Node
from std_srvs.srv import Trigger


class ArmAllNode(Node):
    def __init__(self, robot_indices):
        super().__init__("arm_all_node")
        self.get_logger().info(
            f"Arm All Node Initialized for {', '.join(f'RM{i}' for i in robot_indices)}"
        )
        self.arm_robots(robot_indices)

    def arm_robots(self, robot_indices):
        # Create every client up front so DDS discovery for all robots runs
        # concurrently in the background, instead of paying the discovery
        # latency serially, once per robot.
        clients = {
            f"RM{i}": self.create_client(Trigger, f"/RM{i}/virtual_spacecraft/arm")
            for i in robot_indices
        }
        for robot_name, client in clients.items():
            self.get_logger().info(f"Arming {robot_name} virtual spacecraft...")
            self.call_arm_service(client, f"/{robot_name}/virtual_spacecraft/arm")

    def call_arm_service(self, client, service_name):
        if client.wait_for_service(timeout_sec=10.0):
            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info(f"Successfully armed {service_name}: {result.message}")
            elif result is not None:
                self.get_logger().error(f"Failed to arm {service_name}: {result.message}")
            else:
                self.get_logger().error(f"Failed to arm {service_name}")
        else:
            self.get_logger().warning(f"Service {service_name} not available")


def parse_robot_indices(args):
    parser = argparse.ArgumentParser(
        description="Arm the virtual spacecraft simulation on one or more robots."
    )
    parser.add_argument(
        "robots",
        nargs="*",
        type=int,
        default=list(range(1, 11)),
        help="Robot indices to arm, e.g. '4' or '1 2 4'. Defaults to RM1..RM10.",
    )
    return parser.parse_args(args).robots


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    parsed_args = rclpy.utilities.remove_ros_args(args=args)
    robot_indices = parse_robot_indices(parsed_args[1:])
    node = ArmAllNode(robot_indices)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
