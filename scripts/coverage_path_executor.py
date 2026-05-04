#!/usr/bin/env python3

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


class CoveragePathExecutor(Node):
    def __init__(self):
        super().__init__('coverage_path_executor')

        self.declare_parameter('path_topic', '/room_exploration/coverage_path')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('min_waypoint_spacing', 0.10)
        self.declare_parameter('corner_angle_threshold', 0.35)
        self.declare_parameter('transition_distance', 0.45)
        self.declare_parameter('advance_distance', 0.20)
        self.declare_parameter('final_goal_tolerance', 0.08)
        self.declare_parameter('control_frequency', 5.0)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('goal_clearance_radius', 0.22)
        self.declare_parameter('goal_backoff_distance', 0.28)
        self.declare_parameter('goal_adjustment_step', 0.05)

        self._path_topic = self.get_parameter('path_topic').value
        self._action_name = self.get_parameter('action_name').value
        self._map_frame = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._min_waypoint_spacing = float(self.get_parameter('min_waypoint_spacing').value)
        self._corner_angle_threshold = float(self.get_parameter('corner_angle_threshold').value)
        self._transition_distance = float(self.get_parameter('transition_distance').value)
        self._advance_distance = float(self.get_parameter('advance_distance').value)
        self._final_goal_tolerance = float(self.get_parameter('final_goal_tolerance').value)
        self._control_frequency = float(self.get_parameter('control_frequency').value)
        self._occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self._goal_clearance_radius = float(self.get_parameter('goal_clearance_radius').value)
        self._goal_backoff_distance = float(self.get_parameter('goal_backoff_distance').value)
        self._goal_adjustment_step = float(self.get_parameter('goal_adjustment_step').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav_client = ActionClient(self, NavigateToPose, self._action_name)

        self._goal_sequence = 0
        self._active_waypoints = []
        self._current_index = -1
        self._pending_index = -1
        self._completion_announced = False
        self._last_reported_index = -1
        self._map_msg = None
        self._skip_warn_count = 0

        self.create_subscription(Path, self._path_topic, self._path_callback, 10)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
        self._timer = self.create_timer(1.0 / max(self._control_frequency, 1.0), self._tick)

    def _map_callback(self, msg: OccupancyGrid):
        self._map_msg = msg

    def _path_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn('Received empty coverage path.')
            return

        self._goal_sequence += 1
        self._active_waypoints = self._extract_strip_goals(msg.poses)
        self._current_index = -1
        self._pending_index = -1
        self._completion_announced = False
        self._last_reported_index = -1
        self._skip_warn_count = 0

        self.get_logger().info(
            f'Received coverage path with {len(msg.poses)} poses, '
            f'executing {len(self._active_waypoints)} strip goals'
        )

        self._send_goal_or_skip(0)

    def _extract_strip_goals(self, poses):
        if len(poses) <= 2:
            return list(poses)

        kept = [poses[0]]
        for index in range(1, len(poses) - 1):
            previous_pose = poses[index - 1]
            current_pose = poses[index]
            next_pose = poses[index + 1]

            distance_from_last_kept = self._distance_between(kept[-1], current_pose)
            incoming_distance = self._distance_between(previous_pose, current_pose)
            outgoing_distance = self._distance_between(current_pose, next_pose)

            if distance_from_last_kept < self._min_waypoint_spacing:
                continue

            incoming_heading = self._segment_heading(previous_pose, current_pose)
            outgoing_heading = self._segment_heading(current_pose, next_pose)
            corner_angle = abs(self._normalize_angle(outgoing_heading - incoming_heading))

            transition_detected = (
                incoming_distance >= self._transition_distance
                or outgoing_distance >= self._transition_distance
            )
            corner_detected = corner_angle >= self._corner_angle_threshold

            if transition_detected or corner_detected:
                kept.append(current_pose)

        if self._distance_between(kept[-1], poses[-1]) > 1e-6:
            kept.append(poses[-1])

        refined = self._refine_waypoints(kept)
        return self._dedupe_waypoints(refined)

    def _refine_waypoints(self, waypoints):
        if len(waypoints) <= 1:
            return list(waypoints)

        refined = []
        for index, waypoint in enumerate(waypoints):
            adjusted = self._adjust_waypoint(index, waypoints)
            refined.append(adjusted if adjusted is not None else waypoint)
        return refined

    def _adjust_waypoint(self, index: int, waypoints):
        original = waypoints[index]
        if self._map_msg is None:
            return original

        direction_candidates = []
        if index > 0:
            direction_candidates.append(self._unit_direction(original, waypoints[index - 1]))
        if index < len(waypoints) - 1:
            direction_candidates.append(self._unit_direction(original, waypoints[index + 1]))

        if len(direction_candidates) == 2:
            blended = (
                direction_candidates[0][0] + direction_candidates[1][0],
                direction_candidates[0][1] + direction_candidates[1][1],
            )
            blended_norm = math.hypot(blended[0], blended[1])
            if blended_norm > 1e-6:
                direction_candidates.insert(0, (blended[0] / blended_norm, blended[1] / blended_norm))

        if not direction_candidates:
            return original

        max_steps = max(1, int(math.ceil(self._goal_backoff_distance / max(self._goal_adjustment_step, 1e-3))))
        distances = [
            max(0.0, self._goal_backoff_distance - step * self._goal_adjustment_step)
            for step in range(max_steps + 1)
        ]

        best_candidate = original
        best_clearance = self._clearance_score(original.pose.position.x, original.pose.position.y)

        for distance in distances:
            for direction_x, direction_y in direction_candidates:
                candidate = self._offset_pose(original, direction_x * distance, direction_y * distance)
                if not self._goal_is_traversable(candidate):
                    continue

                clearance = self._clearance_score(
                    candidate.pose.position.x,
                    candidate.pose.position.y,
                )
                if clearance > best_clearance:
                    best_candidate = candidate
                    best_clearance = clearance

            if best_candidate is not original:
                return best_candidate

        return best_candidate

    def _offset_pose(self, pose_stamped: PoseStamped, dx: float, dy: float) -> PoseStamped:
        shifted = PoseStamped()
        shifted.header = pose_stamped.header
        shifted.pose.position.x = pose_stamped.pose.position.x + dx
        shifted.pose.position.y = pose_stamped.pose.position.y + dy
        shifted.pose.position.z = pose_stamped.pose.position.z
        shifted.pose.orientation = Quaternion(
            x=pose_stamped.pose.orientation.x,
            y=pose_stamped.pose.orientation.y,
            z=pose_stamped.pose.orientation.z,
            w=pose_stamped.pose.orientation.w,
        )
        return shifted

    def _clearance_score(self, x: float, y: float) -> float:
        if self._map_msg is None:
            return float('inf')

        info = self._map_msg.info
        map_x = int((x - info.origin.position.x) / info.resolution)
        map_y = int((y - info.origin.position.y) / info.resolution)

        if not (0 <= map_x < info.width and 0 <= map_y < info.height):
            return -1.0

        clearance_cells = max(1, int(math.ceil(self._goal_clearance_radius / info.resolution)))
        best_distance = float('inf')

        for offset_y in range(-clearance_cells, clearance_cells + 1):
            cell_y = map_y + offset_y
            if not (0 <= cell_y < info.height):
                return -1.0

            for offset_x in range(-clearance_cells, clearance_cells + 1):
                cell_x = map_x + offset_x
                if not (0 <= cell_x < info.width):
                    return -1.0

                if offset_x == 0 and offset_y == 0:
                    continue

                value = self._map_msg.data[cell_y * info.width + cell_x]
                if value < 0 or value >= self._occupied_threshold:
                    distance = math.hypot(offset_x, offset_y) * info.resolution
                    best_distance = min(best_distance, distance)

        return best_distance if best_distance < float('inf') else clearance_cells * info.resolution

    def _dedupe_waypoints(self, waypoints):
        if not waypoints:
            return []

        deduped = [waypoints[0]]
        for waypoint in waypoints[1:]:
            if self._distance_between(deduped[-1], waypoint) >= self._min_waypoint_spacing:
                deduped.append(waypoint)

        if len(deduped) == 1 and len(waypoints) > 1:
            deduped.append(waypoints[-1])

        return deduped

    def _tick(self):
        if not self._active_waypoints:
            return

        if self._current_index < 0:
            if self._pending_index >= 0:
                self._send_goal(self._pending_index)
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        target = self._active_waypoints[self._current_index]
        distance_to_target = math.hypot(
            target.pose.position.x - robot_x,
            target.pose.position.y - robot_y,
        )

        final_index = len(self._active_waypoints) - 1
        advance_threshold = self._final_goal_tolerance if self._current_index == final_index else self._advance_distance

        if distance_to_target > advance_threshold:
            return

        if self._current_index == final_index:
            if not self._completion_announced:
                self.get_logger().info('Coverage executor reached final goal tolerance.')
                self._completion_announced = True
            return

        self._send_goal_or_skip(self._current_index + 1)

    def _send_goal_or_skip(self, waypoint_index: int):
        while waypoint_index < len(self._active_waypoints):
            if self._goal_is_traversable(self._active_waypoints[waypoint_index]):
                self._send_goal(waypoint_index)
                return

            if self._skip_warn_count < 10:
                pose = self._active_waypoints[waypoint_index].pose.position
                self.get_logger().warn(
                    f'Skipping coverage goal {waypoint_index + 1}/{len(self._active_waypoints)} '
                    f'because it is outside map bounds or too close to occupied cells '
                    f'at ({pose.x:.2f}, {pose.y:.2f})'
                )
            self._skip_warn_count += 1
            waypoint_index += 1

        if not self._completion_announced:
            self.get_logger().info('Coverage executor exhausted remaining goals.')
            self._completion_announced = True

    def _send_goal(self, waypoint_index: int):
        if waypoint_index >= len(self._active_waypoints):
            return

        if not self._nav_client.wait_for_server(timeout_sec=0.1):
            self._pending_index = waypoint_index
            self.get_logger().info("Waiting for 'navigate_to_pose' action server...")
            return

        self._current_index = waypoint_index
        self._pending_index = -1
        self._completion_announced = False

        if self._last_reported_index != waypoint_index:
            self.get_logger().info(
                f'Coverage executor sending goal {waypoint_index + 1}/{len(self._active_waypoints)}'
            )
            self._last_reported_index = waypoint_index

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header = self._active_waypoints[waypoint_index].header
        goal_msg.pose.pose = self._active_waypoints[waypoint_index].pose
        current_orientation = self._lookup_robot_orientation()
        if current_orientation is not None:
            goal_msg.pose.pose.orientation = current_orientation
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        sequence = self._goal_sequence
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda future, seq=sequence, idx=waypoint_index: self._goal_response_callback(future, seq, idx)
        )

    def _goal_response_callback(self, future, sequence: int, waypoint_index: int):
        goal_handle = future.result()
        if sequence != self._goal_sequence:
            return

        if goal_handle is None or not goal_handle.accepted:
            self._current_index = -1
            self._pending_index = waypoint_index
            self.get_logger().warn(
                f'Coverage goal {waypoint_index + 1} was rejected by Nav2, will retry.'
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, seq=sequence, idx=waypoint_index: self._result_callback(future, seq, idx)
        )

    def _result_callback(self, future, sequence: int, waypoint_index: int):
        if sequence != self._goal_sequence:
            return

        result = future.result()
        if result is None:
            self.get_logger().error('Coverage executor received empty navigate_to_pose result.')
            return

        if waypoint_index != self._current_index:
            return

        status = result.status
        final_index = len(self._active_waypoints) - 1

        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f'Coverage goal {waypoint_index + 1} was aborted by Nav2.')
            if waypoint_index < final_index:
                self._send_goal_or_skip(waypoint_index + 1)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'Coverage goal {waypoint_index + 1} was canceled due to preemption.')
        elif status == GoalStatus.STATUS_SUCCEEDED:
            if waypoint_index == final_index:
                self.get_logger().info('Coverage executor completed final goal.')
                self._completion_announced = True
            else:
                self._send_goal_or_skip(waypoint_index + 1)

    def _lookup_robot_orientation(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
            )
            return transform.transform.rotation
        except TransformException:
            return None

    def _goal_is_traversable(self, pose_stamped: PoseStamped) -> bool:
        if self._map_msg is None:
            return True

        info = self._map_msg.info
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        map_x = int((x - info.origin.position.x) / info.resolution)
        map_y = int((y - info.origin.position.y) / info.resolution)

        if not (0 <= map_x < info.width and 0 <= map_y < info.height):
            return False

        clearance_cells = max(0, int(math.ceil(self._goal_clearance_radius / info.resolution)))
        for offset_y in range(-clearance_cells, clearance_cells + 1):
            cell_y = map_y + offset_y
            if not (0 <= cell_y < info.height):
                return False

            for offset_x in range(-clearance_cells, clearance_cells + 1):
                cell_x = map_x + offset_x
                if not (0 <= cell_x < info.width):
                    return False

                if offset_x * offset_x + offset_y * offset_y > clearance_cells * clearance_cells:
                    continue

                value = self._map_msg.data[cell_y * info.width + cell_x]
                if value < 0 or value >= self._occupied_threshold:
                    return False

        return True

    @staticmethod
    def _distance_between(pose_a: PoseStamped, pose_b: PoseStamped) -> float:
        return math.hypot(
            pose_b.pose.position.x - pose_a.pose.position.x,
            pose_b.pose.position.y - pose_a.pose.position.y,
        )

    @staticmethod
    def _segment_heading(start_pose: PoseStamped, end_pose: PoseStamped) -> float:
        return math.atan2(
            end_pose.pose.position.y - start_pose.pose.position.y,
            end_pose.pose.position.x - start_pose.pose.position.x,
        )

    @staticmethod
    def _unit_direction(start_pose: PoseStamped, end_pose: PoseStamped):
        dx = end_pose.pose.position.x - start_pose.pose.position.x
        dy = end_pose.pose.position.y - start_pose.pose.position.y
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return (0.0, 0.0)
        return (dx / norm, dy / norm)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = CoveragePathExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
