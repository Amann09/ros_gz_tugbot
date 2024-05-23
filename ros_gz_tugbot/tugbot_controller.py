#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

way_points = [(0, 0), (0, 1), (0, 2), (1, 2), (1, 1), (1, 0), (2, 0), (3, 0), 
              (4, 0), (4, 1), (3, 1), (2, 1), (2, 2), (3, 2), (3, 3)]

class TugbotController(Node):
    def __init__(self):
        super().__init__("tugbotController")
        self.get_logger().info("Tugbot Controller is running...")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/model/tugbot/cmd_vel", 10)
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)

    def calculate_yaw(self, initial_coordinate, final_coordinate):
        self.x1, self.y1 = initial_coordinate
        self.x2, self.y2 = final_coordinate
        self.target_yaw = math.atan2(self.y2 - self.y1, self.x2 - self.x1)
        self.angular_z = round(self.target_yaw, 3)
        return self.angular_z

    def send_velocity_command(self):
        msg = Twist()
        i = 0
        for waypoint in way_points[1:]:
            initial_coordinate = way_points[i]
            (x, y) = waypoint
            msg.linear.x = float(x)
            msg.linear.y = float(y)
            msg.angular.z = float(calculate_yaw(initial_coordinate, (x,y)))
            self.cmd_vel_pub_.publish(msg)
            i += 1

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
