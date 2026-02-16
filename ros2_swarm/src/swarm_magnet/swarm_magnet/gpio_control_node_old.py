#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import RPi.GPIO as GPIO

class GPIOControlNode(Node):
    def __init__(self):
        super().__init__('gpio_control_node')
        ####### Use BCM numbering
        GPIO.setmode(GPIO.BCM)  
        self.pin = 4  # GPIO pin number

        ####### Use physical pin numbering`
        # GPIO.setmode(GPIO.BOARD) 
        # self.pin = 7 # GPIO pin number
        
        GPIO.setup(self.pin, GPIO.OUT)
        self.get_logger().info('GPIO Control Node started.')

        # Service to control the GPIO pin
        self.srv = self.create_service(SetBool, 'toggle_magnet', self.handle_toggle)

    def handle_toggle(self, request, response):
        if request.data:
            GPIO.output(self.pin, GPIO.HIGH)
            self.get_logger().info(f'Pin {self.pin} set to HIGH - MAGNET ON')
            response.success = True
            response.message = 'Pin set to HIGH - MAGNET ON'
        else:
            GPIO.output(self.pin, GPIO.LOW)
            self.get_logger().info(f'Pin {self.pin} set to LOW - MAGNET OFF')
            response.success = True
            response.message = 'Pin set to LOW - MAGNET OFF'
        return response

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPIOControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()