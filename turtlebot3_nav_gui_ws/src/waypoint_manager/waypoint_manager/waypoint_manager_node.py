#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)

        # Static waypoints: [x, y, yaw, name]
        self.waypoints = [
            [0.0, 0.0, 0.0, "Home"],
            [2.0, 0.5, 0.0, "Station A"],
            [1.0, -1.5, 0.0, "Station B"],
            [-1.0, 1.0, 0.0, "Station C"],
            [-2.0, -0.5, 0.0, "Station D"],
            [0.0, -2.0, 0.0, "Docking"]
        ]

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, (x, y, yaw, name) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.05
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.text = name
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

