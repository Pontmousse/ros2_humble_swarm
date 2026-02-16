#!/usr/bin/env python3

import glob
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import gpiod


class GPIOControlNode(Node):
    def __init__(self):
        super().__init__('gpio_control_node')

        # ROS parameter (kernel line number!)
        self.declare_parameter('gpio_line', 4)
        self.gpio_line = self.get_parameter('gpio_line').value

        # Select gpiochip (highest index = RP1 on Pi 5)
        chips = glob.glob("/dev/gpiochip*")
        chip_path = max(chips, key=lambda p: int(p.replace("/dev/gpiochip", "")))

        self.get_logger().info(f"Using GPIO chip: {chip_path}")
        self.get_logger().info(f"Using GPIO line: {self.gpio_line}")

        self.chip = gpiod.Chip(chip_path)
        self.line = self.chip.get_line(self.gpio_line)

        self.line.request(
            consumer="ros2_gpio_control",
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[0],
        )

        self.srv = self.create_service(
            SetBool,
            'toggle_magnet',
            self.handle_toggle
        )

        self.get_logger().info("GPIO Control Node started")

    def handle_toggle(self, request, response):
        value = 1 if request.data else 0
        self.line.set_value(value)

        state = "ON" if request.data else "OFF"
        self.get_logger().info(f"GPIO {self.gpio_line} set {state}")

        response.success = True
        response.message = f"Magnet {state}"
        return response

    def destroy_node(self):
        try:
            self.line.set_value(0)
            self.line.release()
            self.chip.close()
        except Exception as e:
            self.get_logger().warn(f"GPIO cleanup error: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPIOControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
