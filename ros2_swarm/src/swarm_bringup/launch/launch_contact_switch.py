"""Pi contact-only mode: just the limit switch, for a centralized run from the dev PC.

One contact_switch_node per robot in ROBOT_NAME, publishing /<robot>/contact_detected.
Deploy with `-e launch_file=launch_contact_switch.py`.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    names = [x.strip() for x in os.environ["ROBOT_NAME"].split(",") if x.strip()]
    gpio_lines = [int(x) for x in os.environ.get("CONTACT_GPIO", "5").split(",") if x.strip()]
    return LaunchDescription([
        Node(package="swarm_magnet", executable="contact_switch_node", name="contact_switch",
             namespace=name, output="screen", emulate_tty=True,
             parameters=[{"gpio_lines": gpio_lines}])
        for name in names])
