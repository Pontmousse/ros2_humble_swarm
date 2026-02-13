# ROS 2 Python Node to Stop All Robots (RM1 to RM10)

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class SafetyStopNode(Node):
    def __init__(self):
        super().__init__('safety_stop_node')
        self.get_logger().info('Safety Stop Node Initialized')
        self.stop_all_robots()

    def stop_all_robots(self):
        for i in range(1, 11):
            robot_name = f'RM{i}'
            service_name = f'/{robot_name}/engage_wheels'
            if self.service_exists(service_name):
                self.get_logger().info(f'Stopping {robot_name}...')
                self.call_stop_service(service_name)
            else:
                self.get_logger().info(f'{robot_name} not active or service not found.')

    def service_exists(self, service_name):
        service_list = self.get_service_names_and_types()
        for name, types in service_list:
            if name == service_name and 'std_srvs/srv/SetBool' in types:
                return True
        return False

    def call_stop_service(self, service_name):
        client = self.create_client(SetBool, service_name)
        if client.wait_for_service(timeout_sec=1.0):
            request = SetBool.Request()
            request.data = False
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f'Successfully stopped {service_name}')
            else:
                self.get_logger().error(f'Failed to stop {service_name}')
        else:
            self.get_logger().warning(f'Service {service_name} not available')

def main(args=None):
    rclpy.init(args=args)
    node = SafetyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
