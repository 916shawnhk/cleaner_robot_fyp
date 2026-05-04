#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


class CoverageProgressTracker(Node):
    def __init__(self):
        super().__init__('coverage_progress_tracker')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('coverage_radius', 0.20)
        self.declare_parameter('reachable_clearance_radius', 0.20)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('publish_topic', '/coverage/traversed_map')
        self.declare_parameter('robot_track_topic', '/coverage/robot_track')
        self.declare_parameter('update_period_sec', 0.4)
        self.declare_parameter('min_pose_separation', 0.03)

        self._map_frame = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._coverage_radius = float(self.get_parameter('coverage_radius').value)
        self._reachable_clearance_radius = float(self.get_parameter('reachable_clearance_radius').value)
        self._occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self._min_pose_separation = float(self.get_parameter('min_pose_separation').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self._map_callback,
            map_qos,
        )

        self._progress_pub = self.create_publisher(OccupancyGrid, self.get_parameter('publish_topic').value, 10)
        self._track_pub = self.create_publisher(Path, self.get_parameter('robot_track_topic').value, 10)

        self._map_msg: Optional[OccupancyGrid] = None
        self._progress_msg: Optional[OccupancyGrid] = None
        self._robot_track = Path()
        self._robot_track.header.frame_id = self._map_frame
        self._last_pose: Optional[PoseStamped] = None

        update_period = float(self.get_parameter('update_period_sec').value)
        self.create_timer(update_period, self._tick)

    def _map_callback(self, msg: OccupancyGrid):
        self._map_msg = msg
        if self._progress_msg is None or self._map_shape_changed(msg):
            self._progress_msg = OccupancyGrid()
            self._progress_msg.header.frame_id = self._map_frame
            self._progress_msg.info = msg.info
            self._progress_msg.data = self._build_reachable_mask(msg)
            self.get_logger().info(
                f'Coverage progress grid initialized at {msg.info.width}x{msg.info.height}'
            )

    def _map_shape_changed(self, msg: OccupancyGrid) -> bool:
        if self._progress_msg is None:
            return True
        info = self._progress_msg.info
        return (
            info.width != msg.info.width
            or info.height != msg.info.height
            or abs(info.resolution - msg.info.resolution) > 1e-9
        )

    def _tick(self):
        if self._map_msg is None or self._progress_msg is None:
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return

        if self._last_pose is not None:
            distance = math.hypot(
                robot_pose.pose.position.x - self._last_pose.pose.position.x,
                robot_pose.pose.position.y - self._last_pose.pose.position.y,
            )
            if distance < self._min_pose_separation:
                self._publish()
                return

        self._last_pose = robot_pose
        self._mark_traversed_area(robot_pose.pose.position.x, robot_pose.pose.position.y)
        self._append_track_pose(robot_pose)
        self._publish()

    def _lookup_robot_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None

        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _mark_traversed_area(self, world_x: float, world_y: float):
        info = self._map_msg.info
        center_x = int((world_x - info.origin.position.x) / info.resolution)
        center_y = int((world_y - info.origin.position.y) / info.resolution)
        radius_cells = max(1, int(math.ceil(self._coverage_radius / info.resolution)))

        for offset_y in range(-radius_cells, radius_cells + 1):
            map_y = center_y + offset_y
            if not (0 <= map_y < info.height):
                continue
            for offset_x in range(-radius_cells, radius_cells + 1):
                map_x = center_x + offset_x
                if not (0 <= map_x < info.width):
                    continue
                if offset_x * offset_x + offset_y * offset_y > radius_cells * radius_cells:
                    continue
                index = map_y * info.width + map_x
                if self._progress_msg.data[index] >= 0:
                    self._progress_msg.data[index] = 100

    def _build_reachable_mask(self, msg: OccupancyGrid):
        info = msg.info
        width = info.width
        height = info.height
        clearance_cells = max(1, int(math.ceil(self._reachable_clearance_radius / info.resolution)))
        mask = [-1] * (width * height)

        for map_y in range(height):
            for map_x in range(width):
                index = map_y * width + map_x
                if self._cell_is_reachable(msg, map_x, map_y, clearance_cells):
                    mask[index] = 0

        return mask

    def _cell_is_reachable(
        self,
        msg: OccupancyGrid,
        map_x: int,
        map_y: int,
        clearance_cells: int,
    ) -> bool:
        info = msg.info
        width = info.width
        height = info.height

        for offset_y in range(-clearance_cells, clearance_cells + 1):
            cell_y = map_y + offset_y
            if not (0 <= cell_y < height):
                return False

            for offset_x in range(-clearance_cells, clearance_cells + 1):
                cell_x = map_x + offset_x
                if not (0 <= cell_x < width):
                    return False
                if offset_x * offset_x + offset_y * offset_y > clearance_cells * clearance_cells:
                    continue

                value = msg.data[cell_y * width + cell_x]
                if value < 0 or value >= self._occupied_threshold:
                    return False

        return True

    def _append_track_pose(self, pose: PoseStamped):
        tracked_pose = PoseStamped()
        tracked_pose.header.frame_id = self._map_frame
        tracked_pose.header.stamp = self.get_clock().now().to_msg()
        tracked_pose.pose = pose.pose
        self._robot_track.header.frame_id = self._map_frame
        self._robot_track.header.stamp = tracked_pose.header.stamp
        self._robot_track.poses.append(tracked_pose)

    def _publish(self):
        self._progress_msg.header.frame_id = self._map_frame
        self._progress_msg.header.stamp = self.get_clock().now().to_msg()
        self._progress_pub.publish(self._progress_msg)
        self._track_pub.publish(self._robot_track)


def main():
    rclpy.init()
    node = CoverageProgressTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
