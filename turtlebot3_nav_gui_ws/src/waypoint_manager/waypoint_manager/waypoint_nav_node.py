#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import math
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_nav_node')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(5.0, self.navigate_waypoints)

        # Define waypoints: [x, y, yaw]
        self.waypoints = [
            [0.0, 0.0, 0.0],
            [1.0, 0.5, 0.0],
            [1.5, -0.5, 0.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0]
        ]
        self.current_index = 0

    def navigate_waypoints(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("✅ All waypoints visited.")
            rclpy.shutdown()
            return

        x, y, yaw = self.waypoints[self.current_index]
        self.get_logger().info(f"Navigating to waypoint {self.current_index + 1}: ({x:.2f}, {y:.2f})")

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(goal)
        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

