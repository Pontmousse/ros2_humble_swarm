# ROS 2 Python Node to Stop All Robots (RM1 to RM10)

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class SafetyStopNode(Node):
    def __init__(self):
        super().__init__('capture_trigger_node')
        self.get_logger().info('Capture Trigger Node Initialized')
        self.all_robots_capture()

    def all_robots_capture(self):
        for i in range(1, 11):
            robot_name = f'RM{i}'
            service_name = f'/{robot_name}/toggle_controller'
            if self.service_exists(service_name):
                self.get_logger().info(f'Switching {robot_name} to Capture mode...')
                self.call_capture_service(service_name)
            else:
                self.get_logger().info(f'{robot_name} not active or service not found.')

    def service_exists(self, service_name):
        service_list = self.get_service_names_and_types()
        for name, types in service_list:
            if name == service_name and 'std_srvs/srv/SetBool' in types:
                return True
        return False

    def call_capture_service(self, service_name):
        client = self.create_client(SetBool, service_name)
        if client.wait_for_service(timeout_sec=1.0):
            request = SetBool.Request()
            request.data = True
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f'Successfully triggered {service_name}')
            else:
                self.get_logger().error(f'Failed to trigger {service_name}')
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
