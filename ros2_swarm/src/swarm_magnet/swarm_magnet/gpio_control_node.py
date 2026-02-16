#!/usr/bin/env python3

import glob
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import gpiod


class GPIOControlNode(Node):
    def __init__(self):
        super().__init__('gpio_control_node')

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter('gpio_line', 4)
        self.gpio_line = (
            self.get_parameter('gpio_line')
            .get_parameter_value()
            .integer_value
        )

        # ----------------------------
        # Select gpiochip (highest index)
        # ----------------------------
        chips = glob.glob("/dev/gpiochip*")
        if not chips:
            raise RuntimeError("No gpiochip devices found")

        chip_path = max(
            chips, key=lambda p: int(p.replace("/dev/gpiochip", ""))
        )
        self.get_logger().info(f"Using GPIO chip: {chip_path}")
        self.get_logger().info(f"Using GPIO line: {self.gpio_line}")

        self.chip = gpiod.Chip(chip_path)

        # ----------------------------
        # Request GPIO line (libgpiod v2)
        # ----------------------------
        self.lines = self.chip.request_lines(
            consumer="ros2_gpio_control",
            config={
                self.gpio_line: gpiod.LineSettings(
                    direction=gpiod.LineDirection.OUTPUT,
                    output_value=gpiod.LineValue.INACTIVE,
                )
            },
        )

        self.get_logger().info(
            f"GPIO Control Node started (line {self.gpio_line})"
        )

        # ----------------------------
        # Service
        # ----------------------------
        self.srv = self.create_service(
            SetBool,
            'toggle_magnet',
            self.handle_toggle
        )

    def handle_toggle(self, request, response):
        value = (
            gpiod.LineValue.ACTIVE
            if request.data
            else gpiod.LineValue.INACTIVE
        )

        self.lines.set_values({self.gpio_line: value})

        state = "ON" if request.data else "OFF"
        self.get_logger().info(
            f"GPIO {self.gpio_line} set {state}"
        )

        response.success = True
        response.message = f"Magnet {state}"
        return response

    def destroy_node(self):
        self.get_logger().info("Releasing GPIO line")
        try:
            self.lines.release()
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
