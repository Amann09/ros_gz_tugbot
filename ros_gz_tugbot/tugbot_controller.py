#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.goal_x = float(5) - 3.82
        self.goal_y = float(2) - 2.09

        self.get_logger().info('Tugbot controller has started...')

        self.subscriber = self.create_subscription(Odometry, '/model/tugbot/odometry', self.pose_callback, 10)
        self.publisher = self.create_publisher(Twist, '/model/tugbot/cmd_vel', 10)

        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.1

        self.kp_angular = 0.5 # earlier 4.0
        self.ki_angular = 0.0
        self.kd_angular = 0.1

        self.prev_error_linear = 0.0
        self.integral_linear = 0.0

        self.prev_error_angular = 0.0
        self.integral_angular = 0.0

    def pose_callback(self, msg: Odometry):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_orientation = msg.pose.pose.orientation
        # current_x = msg.pose.position.x
        # current_y = msg.pose.position.y
        # current_orientation = msg.pose.orientation

        # Debugging information to verify current position and orientation
        self.get_logger().info(f"Current Position: x={current_x}, y={current_y}")
        self.get_logger().info(f"Current Orientation: {current_orientation}")

        goal_angle = math.atan2(self.goal_y - current_y, self.goal_x - current_x)
        current_angle = self.get_yaw_from_quaternion(current_orientation)
        
        error_linear = math.sqrt((self.goal_x - current_x) ** 2 + (self.goal_y - current_y) ** 2)
        error_angular = self.normalize_angle(goal_angle - current_angle)

        # PID for angular velocity
        self.integral_angular += error_angular
        derivative_angular = error_angular - self.prev_error_angular

        angular_speed = (self.kp_angular * error_angular + 
                         self.ki_angular * self.integral_angular + 
                         self.kd_angular * derivative_angular)

        self.prev_error_angular = error_angular

        # Stop if we are close to the goal
        if error_linear < 0.1:
            linear_speed = 0.0
            angular_speed = 0.0
            self.get_logger().info('Goal reached!')
            self.publisher.publish(Twist())
            exit()
            return

        # Publish velocity commands
        cmd_vel = Twist()
        if abs(angular_speed) > 0.01:
            self.get_logger().info('Approaching towards Goal Angle..')
            linear_speed = 0.0
            cmd_vel.linear.x = linear_speed
            cmd_vel.angular.z = angular_speed
            self.publisher.publish(cmd_vel)
        else:
            self.get_logger().info('Approaching towards Goal Position..')

            # PID for linear velocity
            self.integral_linear += error_linear
            derivative_linear = error_linear - self.prev_error_linear

            linear_speed = (self.kp_linear * error_linear + 
                            self.ki_linear * self.integral_linear + 
                            self.kd_linear * derivative_linear)

            cmd_vel.linear.x = linear_speed
            cmd_vel.angular.z = 0.0
            self.publisher.publish(cmd_vel)

            self.prev_error_linear = error_linear

    def get_yaw_from_quaternion(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        if angle > math.pi:
            angle -= 2.0 * math.pi
        elif angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
