"""Publish one natural-language command to /llm_robot/command.

    ros2 run ur3_llm_control send_command "Đưa khối màu đỏ vào vùng B."
"""
import sys
import time

import rclpy
from std_msgs.msg import String


def main(args=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if not argv:
        print('usage: ros2 run ur3_llm_control send_command "<command>"')
        return 1
    rclpy.init(args=args)
    node = rclpy.create_node('llm_command_sender')
    pub = node.create_publisher(String, '/llm_robot/command', 10)
    deadline = time.monotonic() + 5.0
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('warning: llm_robot_node is not running (no subscriber)')
    pub.publish(String(data=' '.join(argv)))
    rclpy.spin_once(node, timeout_sec=0.3)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
