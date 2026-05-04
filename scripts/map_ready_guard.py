#!/usr/bin/env python3
"""
map_ready_guard.py
------------------
Waits until /map is being published, then exits with code 0. Used in the launch
file as an event-triggered gate so that nav2 only starts once the map TF frame
actually exists.

Usage (in launch file):
    Node(package='cleaner_robot_fyp', executable='map_ready_guard', ...)
    Then use OnProcessExit(target_action=guard, on_exit=[nav2_actions])
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                        QoSProfile, QoSReliabilityPolicy)
import sys


class MapReadyGuard(Node):
    def __init__(self):
        super().__init__('map_ready_guard')
        self.get_logger().info('Waiting for /map to be published...')

        # Must match slam_toolbox's TRANSIENT_LOCAL publisher so we receive
        # the retained (latched) map even if it was published before we connect.
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos)

    def _map_callback(self, msg):
        self.get_logger().info('/map received — map frame is ready. Starting nav2.')
        # Destroy sub so we don't process more messages
        self.destroy_subscription(self.sub)
        # Raise SystemExit to stop spinning and exit with code 0
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = MapReadyGuard()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
