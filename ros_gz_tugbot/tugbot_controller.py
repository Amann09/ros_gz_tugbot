#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class TugbotController(Node):
    def __init__(self):
        super().__init__("tugbotController")
        self.get_logger().info("Tugbot Controller is running...")

def main(args=None):
    rclpy.init(args=args)
    node = TugbotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
