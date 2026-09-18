# ROS 2 Python Node to Arm the Virtual Spacecraft Simulation on All Robots (RM1 to RM10)

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ArmAllNode(Node):
    def __init__(self):
        super().__init__("arm_all_node")
        self.get_logger().info("Arm All Node Initialized")
        self.arm_all_robots()

    def arm_all_robots(self):
        for i in range(1, 11):
            robot_name = f"RM{i}"
            service_name = f"/{robot_name}/virtual_spacecraft/arm"
            if self.service_exists(service_name):
                self.get_logger().info(f"Arming {robot_name} virtual spacecraft...")
                self.call_arm_service(service_name)
            else:
                self.get_logger().info(f"{robot_name} not active or service not found.")

    def service_exists(self, service_name):
        service_list = self.get_service_names_and_types()
        for name, types in service_list:
            if name == service_name and "std_srvs/srv/Trigger" in types:
                return True
        return False

    def call_arm_service(self, service_name):
        client = self.create_client(Trigger, service_name)
        if client.wait_for_service(timeout_sec=1.0):
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


def main(args=None):
    rclpy.init(args=args)
    node = ArmAllNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
