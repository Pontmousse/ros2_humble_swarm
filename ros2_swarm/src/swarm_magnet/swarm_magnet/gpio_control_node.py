#!/usr/bin/env python3

import glob
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import gpiod


class GPIOControlNode(Node):
    def __init__(self):
        super().__init__('gpio_control_node')

        # ROS parameter: kernel GPIO line number
        self.declare_parameter('gpio_line', 4)
        self.gpio_line = int(self.get_parameter('gpio_line').value)

        # Detect gpiochip (highest index = RP1 on Pi 5)
        chips = glob.glob("/dev/gpiochip*")
        if not chips:
            raise RuntimeError(
                "No /dev/gpiochip* devices found. "
                "This node must run on GPIO-capable hardware."
            )

        self.chip_path = max(
            chips, key=lambda p: int(p.replace("/dev/gpiochip", ""))
        )

        self.get_logger().info(f"Using GPIO chip: {self.chip_path}")
        self.get_logger().info(f"Using GPIO line: {self.gpio_line}")

        # Request GPIO line using libgpiod v2.4+ API
        self.request = gpiod.request_lines(
            self.chip_path,
            consumer="ros2_gpio_control",
            config={
                self.gpio_line: gpiod.LineSettings(
                    direction=gpiod.line.Direction.OUTPUT,
                    output_value=gpiod.line.Value.INACTIVE,
                )
            },
        )

        self.srv = self.create_service(
            SetBool,
            'toggle_magnet',
            self.handle_toggle
        )

        self.get_logger().info(
            "GPIO Control Node started (libgpiod v2.4+)"
        )

    def handle_toggle(self, request, response):
        value = (
            gpiod.line.Value.ACTIVE
            if request.data
            else gpiod.line.Value.INACTIVE
        )

        self.request.set_values({self.gpio_line: value})

        state = "ON" if request.data else "OFF"
        self.get_logger().info(f"GPIO {self.gpio_line} set {state}")

        response.success = True
        response.message = f"Magnet {state}"
        return response

    def destroy_node(self):
        try:
            # Ensure magnet is off on shutdown
            self.request.set_values(
                {self.gpio_line: gpiod.line.Value.INACTIVE}
            )
            self.request.release()
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
