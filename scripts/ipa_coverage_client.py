#!/usr/bin/env python3

import math

import rclpy
from ipa_building_msgs.action import RoomExploration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener


OCC_GRID_UNKNOWN = -1
OCC_GRID_FREE = 0


class IpaCoverageClient(Node):
    def __init__(self):
        super().__init__('ipa_coverage_client')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('action_name', '/room_exploration/room_exploration_server')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('robot_radius', 0.20)
        self.declare_parameter('coverage_radius', 0.22)
        self.declare_parameter('planning_mode', 1)

        self._map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self._action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self._robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self._coverage_radius = self.get_parameter('coverage_radius').get_parameter_value().double_value
        self._planning_mode = self.get_parameter('planning_mode').get_parameter_value().integer_value

        self._map_msg = None
        self._goal_sent = False
        self._result_future = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._action_client = ActionClient(self, RoomExploration, self._action_name)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._map_sub = self.create_subscription(
            type(self)._resolve_map_type(),
            self._map_topic,
            self._map_callback,
            qos,
        )
        self._timer = self.create_timer(1.0, self._try_send_goal)

    @staticmethod
    def _resolve_map_type():
        from nav_msgs.msg import OccupancyGrid
        return OccupancyGrid

    def _map_callback(self, msg):
        self._map_msg = msg
        self.get_logger().info(
            f'Received map {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m/cell'
        )

    def _try_send_goal(self):
        if self._goal_sent or self._map_msg is None:
            return

        if not self._action_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info('Waiting for IPA room exploration action server...')
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().info(f'Waiting for {self._map_frame}->{self._robot_frame} transform: {exc}')
            return

        goal = RoomExploration.Goal()
        goal.input_map = self._occupancy_grid_to_image(self._map_msg)
        goal.map_resolution = float(self._map_msg.info.resolution)
        goal.map_origin = self._map_msg.info.origin
        goal.robot_radius = float(self._robot_radius)
        goal.coverage_radius = float(self._coverage_radius)
        goal.field_of_view = []
        goal.field_of_view_origin.x = 0.0
        goal.field_of_view_origin.y = 0.0
        goal.field_of_view_origin.z = 0.0
        goal.starting_position.x = float(transform.transform.translation.x)
        goal.starting_position.y = float(transform.transform.translation.y)
        goal.starting_position.theta = self._yaw_from_quaternion(transform.transform.rotation)
        goal.planning_mode = int(self._planning_mode)

        self.get_logger().info(
            f'Sending IPA coverage goal from ({goal.starting_position.x:.2f}, '
            f'{goal.starting_position.y:.2f}, {goal.starting_position.theta:.2f})'
        )
        self._goal_sent = True
        send_goal_future = self._action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('IPA coverage goal was rejected')
            self._goal_sent = False
            return

        self.get_logger().info('IPA coverage goal accepted')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        if result is None:
            self.get_logger().error('IPA coverage result was empty')
            return

        self.get_logger().info(
            f'IPA coverage path ready with {len(result.result.coverage_path_pose_stamped)} poses'
        )

    def _occupancy_grid_to_image(self, map_msg):
        width = map_msg.info.width
        height = map_msg.info.height
        data = bytearray(width * height)

        for index, value in enumerate(map_msg.data):
            if value == OCC_GRID_UNKNOWN:
                data[index] = 127
            elif value == OCC_GRID_FREE:
                data[index] = 255
            else:
                data[index] = 0

        image = Image()
        image.header = map_msg.header
        image.height = height
        image.width = width
        image.encoding = 'mono8'
        image.is_bigendian = 0
        image.step = width
        image.data = bytes(data)
        return image

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main():
    rclpy.init()
    node = IpaCoverageClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
