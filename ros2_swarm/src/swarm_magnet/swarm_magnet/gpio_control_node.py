#!/usr/bin/env python3

import glob
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import gpiod


class GPIOControlNode(Node):
    def __init__(self):
        super().__init__('gpio_control_node')

        # BCM GPIO number
        self.gpio_line = 4

        # Auto-detect gpiochip (works on Pi 4 and Pi 5)
        chip_path = sorted(glob.glob("/dev/gpiochip*"))[-1]
        self.get_logger().info(f"Using GPIO chip: {chip_path}")

        self.chip = gpiod.Chip(chip_path)
        self.line = self.chip.get_line(self.gpio_line)

        self.line.request(
            consumer="ros2_gpio_control",
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[0]
        )

        self.get_logger().info(
            f"GPIO Control Node started (BCM GPIO {self.gpio_line})"
        )

        self.srv = self.create_service(
            SetBool,
            'toggle_magnet',
            self.handle_toggle
        )

    def handle_toggle(self, request, response):
        if request.data:
            self.line.set_value(1)
            self.get_logger().info(
                f"GPIO {self.gpio_line} set HIGH - MAGNET ON"
            )
            response.success = True
            response.message = 'Magnet ON'
        else:
            self.line.set_value(0)
            self.get_logger().info(
                f"GPIO {self.gpio_line} set LOW - MAGNET OFF"
            )
            response.success = True
            response.message = 'Magnet OFF'

        return response

    def destroy_node(self):
        self.get_logger().info("Releasing GPIO line")
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
