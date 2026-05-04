#!/usr/bin/env python3

from collections import deque
import math
import time
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from ipa_building_msgs.action import RoomExploration
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, DockRobot, NavigateThroughPoses, UndockRobot
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


OCC_GRID_UNKNOWN = -1
OCC_GRID_FREE = 0


class CoverageManager(Node):
    def __init__(self):
        super().__init__('coverage_manager')

        self.declare_parameter('autostart', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('exploration_action_name', '/room_exploration/room_exploration_server')
        self.declare_parameter('nav_action_name', 'navigate_through_poses')
        self.declare_parameter('coverage_behavior_tree', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('coverage_radius', 0.20)
        self.declare_parameter('planning_mode', 1)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('goal_clearance_radius', 0.24)
        self.declare_parameter('goal_backoff_distance', 0.30)
        self.declare_parameter('goal_adjustment_step', 0.05)
        self.declare_parameter('min_waypoint_spacing', 0.12)
        self.declare_parameter('segment_sample_distance', 0.30)
        self.declare_parameter('corner_angle_threshold', 0.45)
        self.declare_parameter('lateral_deviation_tolerance', 0.05)
        self.declare_parameter('execution_chunk_size', 2)
        self.declare_parameter('execution_chunk_distance', 0.8)
        self.declare_parameter('min_send_goal_distance', 0.25)
        self.declare_parameter('close_goal_skip_distance', 0.45)
        self.declare_parameter('close_goal_forward_angle', 1.05)
        self.declare_parameter('stall_timeout_sec', 12.0)
        self.declare_parameter('stall_progress_epsilon', 0.15)
        self.declare_parameter('max_recoveries_per_chunk', 8)
        self.declare_parameter('start_near_robot_distance', 0.50)
        self.declare_parameter('always_start_from_beginning', True)
        self.declare_parameter('current_goal_topic', '/coverage/current_goal')
        self.declare_parameter('raw_path_topic', '/coverage/raw_path')
        self.declare_parameter('filtered_path_topic', '/coverage/filtered_path')
        self.declare_parameter('markers_topic', '/coverage/markers')
        self.declare_parameter('progress_topic', '/coverage/traversed_map')
        self.declare_parameter('desired_coverage_ratio', 0.90)
        self.declare_parameter('min_uncovered_region_cells', 80)
        self.declare_parameter('max_supplemental_passes', 3)
        self.declare_parameter('max_supplemental_goals_per_pass', 12)
        self.declare_parameter('supplemental_progress_epsilon', 0.001)
        self.declare_parameter('max_supplemental_cycles_without_progress', 5)
        self.declare_parameter('supplemental_sweep_spacing', 0.16)
        self.declare_parameter('supplemental_min_sweep_length', 0.30)
        self.declare_parameter('launch_start_wall_time', 0.0)
        self.declare_parameter('movement_start_distance_threshold', 0.05)
        self.declare_parameter('enable_escape_reposition', False)
        self.declare_parameter('escape_reposition_min_distance', 0.35)
        self.declare_parameter('escape_reposition_lookback_waypoints', 20)
        self.declare_parameter('max_single_goal_retries_before_defer', 2)
        self.declare_parameter('max_deferred_goal_retries', 3)
        self.declare_parameter('auto_dock_on_completion', False)
        self.declare_parameter('dock_action_name', '/dock_robot')
        self.declare_parameter('dock_id', 'charging_dock')
        self.declare_parameter('dock_max_staging_time', 1000.0)
        self.declare_parameter('dock_navigate_to_staging_pose', True)
        self.declare_parameter('undock_on_start', True)
        self.declare_parameter('undock_action_name', '/undock_robot')
        self.declare_parameter('undock_dock_type', 'simple_charging_dock')
        self.declare_parameter('undock_max_time', 10.0)
        self.declare_parameter('undock_server_wait_timeout', 8.0)
        self.declare_parameter('continue_on_undock_failure', True)
        self.declare_parameter('manual_undock_backup_on_failure', True)
        self.declare_parameter('manual_undock_backup_action_name', '/backup')
        self.declare_parameter('manual_undock_backup_distance', 0.40)
        self.declare_parameter('manual_undock_backup_speed', 0.08)
        self.declare_parameter('manual_undock_backup_time_allowance', 8.0)
        self.declare_parameter('undock_lifecycle_node', 'docking_server')
        self.declare_parameter(
            'nav_ready_lifecycle_nodes',
            ['controller_server', 'planner_server', 'bt_navigator'],
        )

        self._autostart = bool(self.get_parameter('autostart').value)
        self._map_topic = self.get_parameter('map_topic').value
        self._exploration_action_name = self.get_parameter('exploration_action_name').value
        self._nav_action_name = self.get_parameter('nav_action_name').value
        self._coverage_behavior_tree = self.get_parameter('coverage_behavior_tree').value
        self._map_frame = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._robot_radius = float(self.get_parameter('robot_radius').value)
        self._coverage_radius = float(self.get_parameter('coverage_radius').value)
        self._planning_mode = int(self.get_parameter('planning_mode').value)
        self._occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self._goal_clearance_radius = float(self.get_parameter('goal_clearance_radius').value)
        self._goal_backoff_distance = float(self.get_parameter('goal_backoff_distance').value)
        self._goal_adjustment_step = float(self.get_parameter('goal_adjustment_step').value)
        self._min_waypoint_spacing = float(self.get_parameter('min_waypoint_spacing').value)
        self._segment_sample_distance = float(self.get_parameter('segment_sample_distance').value)
        self._corner_angle_threshold = float(self.get_parameter('corner_angle_threshold').value)
        self._lateral_deviation_tolerance = float(self.get_parameter('lateral_deviation_tolerance').value)
        self._execution_chunk_size = max(1, int(self.get_parameter('execution_chunk_size').value))
        self._execution_chunk_distance = float(self.get_parameter('execution_chunk_distance').value)
        self._min_send_goal_distance = float(self.get_parameter('min_send_goal_distance').value)
        self._close_goal_skip_distance = float(self.get_parameter('close_goal_skip_distance').value)
        self._close_goal_forward_angle = float(self.get_parameter('close_goal_forward_angle').value)
        self._stall_timeout_sec = float(self.get_parameter('stall_timeout_sec').value)
        self._stall_progress_epsilon = float(self.get_parameter('stall_progress_epsilon').value)
        self._max_recoveries_per_chunk = int(self.get_parameter('max_recoveries_per_chunk').value)
        self._start_near_robot_distance = float(self.get_parameter('start_near_robot_distance').value)
        self._always_start_from_beginning = bool(self.get_parameter('always_start_from_beginning').value)
        self._desired_coverage_ratio = float(self.get_parameter('desired_coverage_ratio').value)
        self._min_uncovered_region_cells = int(self.get_parameter('min_uncovered_region_cells').value)
        self._max_supplemental_passes = int(self.get_parameter('max_supplemental_passes').value)
        self._max_supplemental_goals_per_pass = int(self.get_parameter('max_supplemental_goals_per_pass').value)
        self._supplemental_progress_epsilon = float(self.get_parameter('supplemental_progress_epsilon').value)
        self._max_supplemental_cycles_without_progress = int(
            self.get_parameter('max_supplemental_cycles_without_progress').value
        )
        self._supplemental_sweep_spacing = float(self.get_parameter('supplemental_sweep_spacing').value)
        self._supplemental_min_sweep_length = float(self.get_parameter('supplemental_min_sweep_length').value)
        self._launch_start_wall_time = float(self.get_parameter('launch_start_wall_time').value)
        self._movement_start_distance_threshold = float(
            self.get_parameter('movement_start_distance_threshold').value
        )
        self._enable_escape_reposition = bool(self.get_parameter('enable_escape_reposition').value)
        self._escape_reposition_min_distance = float(
            self.get_parameter('escape_reposition_min_distance').value
        )
        self._escape_reposition_lookback_waypoints = int(
            self.get_parameter('escape_reposition_lookback_waypoints').value
        )
        self._max_single_goal_retries_before_defer = int(
            self.get_parameter('max_single_goal_retries_before_defer').value
        )
        self._max_deferred_goal_retries = int(self.get_parameter('max_deferred_goal_retries').value)
        self._auto_dock_on_completion = bool(self.get_parameter('auto_dock_on_completion').value)
        self._dock_action_name = self.get_parameter('dock_action_name').value
        self._dock_id = self.get_parameter('dock_id').value
        self._dock_max_staging_time = float(self.get_parameter('dock_max_staging_time').value)
        self._dock_navigate_to_staging_pose = bool(
            self.get_parameter('dock_navigate_to_staging_pose').value
        )
        self._undock_on_start = bool(self.get_parameter('undock_on_start').value)
        self._undock_action_name = self.get_parameter('undock_action_name').value
        self._undock_dock_type = self.get_parameter('undock_dock_type').value
        self._undock_max_time = float(self.get_parameter('undock_max_time').value)
        self._undock_server_wait_timeout = float(
            self.get_parameter('undock_server_wait_timeout').value
        )
        self._continue_on_undock_failure = bool(
            self.get_parameter('continue_on_undock_failure').value
        )
        self._manual_undock_backup_on_failure = bool(
            self.get_parameter('manual_undock_backup_on_failure').value
        )
        self._manual_undock_backup_action_name = self.get_parameter(
            'manual_undock_backup_action_name'
        ).value
        self._manual_undock_backup_distance = float(
            self.get_parameter('manual_undock_backup_distance').value
        )
        self._manual_undock_backup_speed = float(
            self.get_parameter('manual_undock_backup_speed').value
        )
        self._manual_undock_backup_time_allowance = float(
            self.get_parameter('manual_undock_backup_time_allowance').value
        )
        self._undock_lifecycle_node = self.get_parameter('undock_lifecycle_node').value
        self._nav_ready_lifecycle_nodes = list(self.get_parameter('nav_ready_lifecycle_nodes').value)
        self._node_wall_start_monotonic = time.monotonic()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._exploration_client = ActionClient(self, RoomExploration, self._exploration_action_name)
        self._nav_client = ActionClient(self, NavigateThroughPoses, self._nav_action_name)
        self._dock_client = ActionClient(self, DockRobot, self._dock_action_name)
        self._undock_client = ActionClient(self, UndockRobot, self._undock_action_name)
        self._manual_undock_backup_client = ActionClient(
            self,
            BackUp,
            self._manual_undock_backup_action_name,
        )
        self._lifecycle_clients = {}
        self._lifecycle_futures = {}
        self._lifecycle_active_nodes = set()
        self._last_lifecycle_wait_log_monotonic = {}

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, self._map_topic, self._map_callback, map_qos)
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('progress_topic').value,
            self._progress_callback,
            10,
        )

        self._raw_path_pub = self.create_publisher(Path, self.get_parameter('raw_path_topic').value, 10)
        self._filtered_path_pub = self.create_publisher(Path, self.get_parameter('filtered_path_topic').value, 10)
        self._current_goal_pub = self.create_publisher(PoseStamped, self.get_parameter('current_goal_topic').value, 10)
        self._markers_pub = self.create_publisher(MarkerArray, self.get_parameter('markers_topic').value, 10)

        self._map_msg: Optional[OccupancyGrid] = None
        self._progress_msg: Optional[OccupancyGrid] = None
        self._request_sent = False
        self._plan_ready = False
        self._execution_complete = False
        self._goal_cancel_in_progress = False
        self._goal_handle = None
        self._dock_goal_handle = None
        self._dock_request_sent = False
        self._dock_complete = False
        self._undock_goal_handle = None
        self._undock_request_sent = False
        self._undock_complete = not self._undock_on_start
        self._manual_undock_goal_handle = None
        self._manual_undock_request_sent = False
        self._undock_wait_start_monotonic: Optional[float] = None
        self._last_undock_wait_log_monotonic = 0.0

        self._raw_path: List[PoseStamped] = []
        self._filtered_path: List[PoseStamped] = []
        self._supplemental_pass_count = 0

        self._next_index = 0
        self._active_chunk_start = -1
        self._active_chunk_end = -1
        self._active_feedback_goal_index = -1
        self._active_is_deferred = False
        self._active_is_escape = False
        self._chunk_retry_count = 0
        self._chunk_distance_remaining = None
        self._cancel_failure_reason = None
        self._last_progress_time = None
        self._last_feedback_time = None
        self._last_feedback_recoveries = 0
        self._last_exhausted_coverage_ratio = None
        self._supplemental_cycles_without_progress = 0
        self._single_goal_mode = False
        self._retry_goal_override: Optional[PoseStamped] = None
        self._deferred_indices = deque()
        self._deferred_index_set = set()
        self._abandoned_indices = set()
        self._movement_start_wall_time: Optional[float] = None
        self._movement_reference_pose: Optional[PoseStamped] = None
        self._pending_escape_goal: Optional[PoseStamped] = None
        self._pending_escape_resume_index = 0
        self._last_completed_index = -1

        self.create_timer(0.5, self._tick)

    def _map_callback(self, msg: OccupancyGrid):
        self._map_msg = msg

    def _progress_callback(self, msg: OccupancyGrid):
        self._progress_msg = msg

    def _tick(self):
        if self._autostart and not self._request_sent:
            if not self._undock_start_gate_ready():
                return
            self._try_send_planning_request()
            return

        if self._goal_handle is not None:
            self._monitor_active_chunk()
            return

        if self._plan_ready and not self._execution_complete and not self._goal_cancel_in_progress:
            self._send_next_chunk()
            return

        if self._execution_complete and self._auto_dock_on_completion and not self._dock_complete:
            self._try_send_dock_request()

    def _undock_start_gate_ready(self) -> bool:
        if self._undock_complete:
            return True

        if self._undock_request_sent:
            return False

        now_monotonic = time.monotonic()
        if self._undock_wait_start_monotonic is None:
            self._undock_wait_start_monotonic = now_monotonic

        if not self._lifecycle_nodes_active([self._undock_lifecycle_node], 'undock'):
            return False

        if not self._undock_client.wait_for_server(timeout_sec=0.1):
            elapsed = now_monotonic - self._undock_wait_start_monotonic
            if elapsed >= self._undock_server_wait_timeout:
                self.get_logger().warn(
                    f"Undock action server '{self._undock_action_name}' was not available "
                    f"after {elapsed:.1f}s; starting coverage without undocking"
                )
                self._undock_complete = True
                return True

            if now_monotonic - self._last_undock_wait_log_monotonic >= 2.0:
                self._last_undock_wait_log_monotonic = now_monotonic
                self.get_logger().info(
                    f"Waiting for undock action server '{self._undock_action_name}'..."
                )
            return False

        goal = UndockRobot.Goal()
        goal.dock_type = self._undock_dock_type
        goal.max_undocking_time = float(self._undock_max_time)

        self._undock_request_sent = True
        self.get_logger().info(
            f"Sending undock request before coverage "
            f"(dock_type='{goal.dock_type}', max_time={goal.max_undocking_time:.1f}s)"
        )
        send_future = self._undock_client.send_goal_async(goal)
        send_future.add_done_callback(self._undock_goal_response_callback)
        return False

    def _undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._handle_undock_failure(
                'Undock request was rejected after docking_server became active'
            )
            return

        self._undock_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._undock_result_callback)

    def _undock_result_callback(self, future):
        result = future.result()
        self._undock_goal_handle = None

        if result is None:
            self.get_logger().warn('Undock finished with an empty result; starting coverage anyway')
            self._undock_complete = True
            return

        undock_result = result.result
        if result.status == GoalStatus.STATUS_SUCCEEDED and undock_result.success:
            self.get_logger().info('Undock complete; starting coverage planning')
            self._undock_complete = True
            return

        message = (
            f'Undock did not complete before coverage: status={result.status}, '
            f'error_code={undock_result.error_code}, message="{undock_result.error_msg}"'
        )
        self._handle_undock_failure(message)

    def _handle_undock_failure(self, message: str):
        if self._manual_undock_backup_on_failure and not self._manual_undock_request_sent:
            self.get_logger().warn(
                f'{message}; OpenNav thinks the robot is not docked, so forcing a short '
                'manual back-out before coverage'
            )
            self._try_send_manual_undock_backup()
            return

        if self._continue_on_undock_failure:
            self.get_logger().warn(f'{message}; continuing coverage without a back-out')
            self._undock_complete = True
        else:
            self.get_logger().error(f'{message}; coverage autostart is blocked')

    def _try_send_manual_undock_backup(self):
        if not self._manual_undock_backup_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"Manual undock backup action server '{self._manual_undock_backup_action_name}' "
                'is unavailable; continuing coverage without a back-out'
            )
            self._undock_complete = True
            return

        goal = BackUp.Goal()
        goal.target = Point(x=-abs(self._manual_undock_backup_distance), y=0.0, z=0.0)
        goal.speed = abs(self._manual_undock_backup_speed)
        goal.time_allowance = Duration(
            seconds=self._manual_undock_backup_time_allowance
        ).to_msg()

        self._manual_undock_request_sent = True
        self.get_logger().info(
            f"Sending manual undock backup "
            f"({abs(goal.target.x):.2f}m at {goal.speed:.2f}m/s)"
        )
        send_future = self._manual_undock_backup_client.send_goal_async(goal)
        send_future.add_done_callback(self._manual_undock_goal_response_callback)

    def _manual_undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(
                'Manual undock backup was rejected; continuing coverage without a back-out'
            )
            self._undock_complete = True
            return

        self._manual_undock_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._manual_undock_result_callback)

    def _manual_undock_result_callback(self, future):
        result = future.result()
        self._manual_undock_goal_handle = None

        if result is None:
            self.get_logger().warn(
                'Manual undock backup finished with an empty result; starting coverage anyway'
            )
            self._undock_complete = True
            return

        backup_result = result.result
        if result.status == GoalStatus.STATUS_SUCCEEDED and backup_result.error_code == BackUp.Result.NONE:
            self.get_logger().info('Manual undock backup complete; starting coverage planning')
        else:
            self.get_logger().warn(
                f'Manual undock backup did not complete cleanly: status={result.status}, '
                f'error_code={backup_result.error_code}, message="{backup_result.error_msg}"; '
                'starting coverage anyway'
            )
        self._undock_complete = True

    def _try_send_planning_request(self):
        if self._map_msg is None:
            return

        if not self._lifecycle_nodes_active(self._nav_ready_lifecycle_nodes, 'coverage navigation'):
            return

        if not self._exploration_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info('Waiting for IPA room exploration action server...')
            return

        if not self._nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info(f"Waiting for Nav2 '{self._nav_action_name}' action server...")
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
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
        goal.starting_position.x = robot_pose.pose.position.x
        goal.starting_position.y = robot_pose.pose.position.y
        goal.starting_position.theta = self._yaw_from_quaternion(robot_pose.pose.orientation)
        goal.planning_mode = int(self._planning_mode)

        self._request_sent = True
        self.get_logger().info(
            f'Requesting coverage plan from ({goal.starting_position.x:.2f}, '
            f'{goal.starting_position.y:.2f}, {goal.starting_position.theta:.2f})'
        )
        send_future = self._exploration_client.send_goal_async(goal)
        send_future.add_done_callback(self._planning_goal_response_callback)

    def _lifecycle_nodes_active(self, node_names: List[str], purpose: str) -> bool:
        for node_name in node_names:
            if not self._lifecycle_node_active(node_name, purpose):
                return False
        return True

    def _lifecycle_node_active(self, node_name: str, purpose: str) -> bool:
        if node_name in self._lifecycle_active_nodes:
            return True

        client = self._lifecycle_clients.get(node_name)
        if client is None:
            client = self.create_client(GetState, f'/{node_name}/get_state')
            self._lifecycle_clients[node_name] = client

        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.1):
                self._log_lifecycle_wait(node_name, purpose, 'service unavailable')
                return False

        future = self._lifecycle_futures.get(node_name)
        if future is not None:
            if not future.done():
                return False

            try:
                response = future.result()
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to query lifecycle state for '{node_name}': {exc}"
                )
                self._lifecycle_futures.pop(node_name, None)
                return False

            self._lifecycle_futures.pop(node_name, None)
            state = response.current_state
            if state.id == State.PRIMARY_STATE_ACTIVE:
                self._lifecycle_active_nodes.add(node_name)
                return True

            self._log_lifecycle_wait(node_name, purpose, f"state={state.label or state.id}")

        self._lifecycle_futures[node_name] = client.call_async(GetState.Request())
        return False

    def _log_lifecycle_wait(self, node_name: str, purpose: str, detail: str):
        now_monotonic = time.monotonic()
        last_log = self._last_lifecycle_wait_log_monotonic.get(node_name, 0.0)
        if now_monotonic - last_log < 2.0:
            return

        self._last_lifecycle_wait_log_monotonic[node_name] = now_monotonic
        self.get_logger().info(
            f"Waiting for lifecycle node '{node_name}' to become active before {purpose} "
            f"({detail})"
        )

    def _planning_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Coverage planning request was rejected')
            self._request_sent = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._planning_result_callback)

    def _planning_result_callback(self, future):
        result = future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('Coverage planning failed to return a usable result')
            return

        if self._plan_ready or self._goal_handle is not None or self._execution_complete:
            self.get_logger().warn('Ignoring duplicate coverage planning result after mission execution started')
            return

        raw_path = list(result.result.coverage_path_pose_stamped)
        if not raw_path:
            self.get_logger().error('Coverage planner returned an empty path')
            return

        self._raw_path = self._normalize_path(raw_path)
        self._filtered_path = self._build_execution_path(self._raw_path)

        self._publish_path(self._raw_path_pub, self._raw_path)
        self._publish_path(self._filtered_path_pub, self._filtered_path)
        self._publish_markers()

        if not self._filtered_path:
            self.get_logger().error('Coverage path cleanup removed every waypoint')
            return

        self._next_index = self._select_start_index()
        self._plan_ready = True
        self._execution_complete = False
        self.get_logger().info(
            f'Coverage plan ready: {len(self._raw_path)} raw poses -> '
            f'{len(self._filtered_path)} execution waypoints, starting at index {self._next_index + 1}'
        )

    def _normalize_path(self, poses: List[PoseStamped]) -> List[PoseStamped]:
        normalized = []
        for pose in poses:
            normalized_pose = PoseStamped()
            normalized_pose.header.frame_id = self._map_frame
            normalized_pose.header.stamp = self.get_clock().now().to_msg()
            normalized_pose.pose = pose.pose
            normalized.append(normalized_pose)
        return self._orient_path(normalized)

    def _build_execution_path(self, raw_path: List[PoseStamped]) -> List[PoseStamped]:
        if len(raw_path) <= 2:
            return self._orient_path(raw_path)

        filtered = [raw_path[0]]
        segment_distance = 0.0

        for index in range(1, len(raw_path) - 1):
            previous_pose = raw_path[index - 1]
            current_pose = raw_path[index]
            next_pose = raw_path[index + 1]
            last_kept_pose = filtered[-1]

            if self._distance_between(last_kept_pose, current_pose) < self._min_waypoint_spacing:
                continue

            segment_distance += self._distance_between(previous_pose, current_pose)
            incoming_heading = self._segment_heading(previous_pose, current_pose)
            outgoing_heading = self._segment_heading(current_pose, next_pose)
            corner_angle = abs(self._normalize_angle(outgoing_heading - incoming_heading))
            lateral_deviation = self._point_to_segment_distance(
                current_pose,
                last_kept_pose,
                next_pose,
            )

            if (
                corner_angle >= self._corner_angle_threshold
                or segment_distance >= self._segment_sample_distance
                or lateral_deviation >= self._lateral_deviation_tolerance
            ):
                filtered.append(current_pose)
                segment_distance = 0.0

        if self._distance_between(filtered[-1], raw_path[-1]) > 1e-6:
            filtered.append(raw_path[-1])

        adjusted = []
        for index in range(len(filtered)):
            adjusted_pose = self._adjust_waypoint(index, filtered)
            if adjusted_pose is not None and self._goal_is_traversable(adjusted_pose):
                adjusted.append(adjusted_pose)

        if not adjusted:
            adjusted = filtered

        deduped = [adjusted[0]]
        for pose in adjusted[1:]:
            if self._distance_between(deduped[-1], pose) >= self._min_waypoint_spacing:
                deduped.append(pose)

        if len(deduped) == 1 and len(adjusted) > 1:
            deduped.append(adjusted[-1])

        return self._orient_path(deduped)

    def _send_next_chunk(self):
        if not self._filtered_path:
            self._finish_or_extend_coverage()
            return

        if self._pending_escape_goal is not None:
            resume_index = min(max(self._pending_escape_resume_index, 0), len(self._filtered_path) - 1)
            self._send_goal_chunk(
                [self._pending_escape_goal],
                resume_index,
                resume_index,
                is_deferred=False,
                is_escape=True,
            )
            return

        if self._next_index >= len(self._filtered_path):
            self._finish_or_extend_coverage()
            return

        if self._skip_nearby_waypoints():
            if self._next_index >= len(self._filtered_path):
                self._finish_or_extend_coverage()
            else:
                self._publish_markers()
                self._send_next_chunk()
            return

        start_index = self._next_index
        if self._single_goal_mode:
            chunk_poses = [self._goal_for_index(start_index)]
            end_index = start_index
        else:
            chunk_poses = [self._filtered_path[start_index]]
            chunk_distance = 0.0
            end_index = start_index

            while end_index + 1 < len(self._filtered_path):
                next_distance = self._distance_between(self._filtered_path[end_index], self._filtered_path[end_index + 1])
                next_index = end_index + 1
                if len(chunk_poses) >= self._execution_chunk_size:
                    break
                if chunk_distance > 0.0 and chunk_distance + next_distance > self._execution_chunk_distance:
                    break
                chunk_poses.append(self._filtered_path[next_index])
                chunk_distance += next_distance
                end_index = next_index

        self._send_goal_chunk(chunk_poses, start_index, end_index, is_deferred=False, is_escape=False)

    def _skip_nearby_waypoints(self) -> bool:
        if self._min_send_goal_distance <= 0.0:
            return False

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return False

        skipped = 0
        while self._next_index < len(self._filtered_path):
            waypoint = self._filtered_path[self._next_index]
            distance = self._distance_between(robot_pose, waypoint)
            if distance >= self._min_send_goal_distance and not self._is_close_side_or_rear_goal(
                robot_pose,
                waypoint,
                distance,
            ):
                break
            self._last_completed_index = max(self._last_completed_index, self._next_index)
            self._next_index += 1
            skipped += 1

        if skipped:
            self.get_logger().info(
                f'Skipped {skipped} coverage waypoint(s) already within '
                f'{self._min_send_goal_distance:.2f} m of the robot; '
                f'next waypoint index {self._next_index + 1}'
            )
            self._active_feedback_goal_index = self._next_index
            return True

        return False

    def _is_close_side_or_rear_goal(
        self,
        robot_pose: PoseStamped,
        waypoint: PoseStamped,
        distance: float,
    ) -> bool:
        if self._close_goal_skip_distance <= 0.0 or distance >= self._close_goal_skip_distance:
            return False

        robot_yaw = self._yaw_from_quaternion(robot_pose.pose.orientation)
        heading_to_goal = math.atan2(
            waypoint.pose.position.y - robot_pose.pose.position.y,
            waypoint.pose.position.x - robot_pose.pose.position.x,
        )
        relative_heading = abs(self._normalize_angle(heading_to_goal - robot_yaw))
        return relative_heading > self._close_goal_forward_angle

    def _send_deferred_goal(self):
        while self._deferred_indices and self._deferred_indices[0] in self._abandoned_indices:
            abandoned_index = self._deferred_indices.popleft()
            self._deferred_index_set.discard(abandoned_index)

        if not self._deferred_indices:
            self._finish_or_extend_coverage()
            return

        start_index = self._deferred_indices[0]
        chunk_poses = [self._goal_for_index(start_index)]
        self._send_goal_chunk(chunk_poses, start_index, start_index, is_deferred=True, is_escape=False)

    def _send_goal_chunk(
        self,
        chunk_poses: List[PoseStamped],
        start_index: int,
        end_index: int,
        is_deferred: bool,
        is_escape: bool,
    ):
        chunk_distance = 0.0
        if len(chunk_poses) > 1:
            for idx in range(1, len(chunk_poses)):
                chunk_distance += self._distance_between(chunk_poses[idx - 1], chunk_poses[idx])

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.behavior_tree = self._coverage_behavior_tree
        goal_msg.poses = []
        now_msg = self.get_clock().now().to_msg()
        for pose in chunk_poses:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self._map_frame
            goal_pose.header.stamp = now_msg
            goal_pose.pose = pose.pose
            goal_msg.poses.append(goal_pose)

        self._active_chunk_start = start_index
        self._active_chunk_end = end_index
        self._active_feedback_goal_index = start_index
        self._active_is_deferred = is_deferred
        self._active_is_escape = is_escape
        self._chunk_distance_remaining = None
        if self._movement_start_wall_time is None and self._movement_reference_pose is None:
            self._movement_reference_pose = self._lookup_robot_pose()
        now = self.get_clock().now()
        self._last_progress_time = now
        self._last_feedback_time = now
        self._last_feedback_recoveries = 0

        self._publish_current_goal(chunk_poses[0])
        self._publish_markers()
        if is_escape:
            self.get_logger().info(
                f'Sending escape reposition goal before resuming at waypoint index {start_index + 1} '
                f'(retry {self._chunk_retry_count})'
            )
        else:
            self.get_logger().info(
                f'Sending {"deferred goal" if is_deferred else "coverage chunk"} '
                f'{start_index + 1}-{end_index + 1} of {len(self._filtered_path)} '
                f'(retry {self._chunk_retry_count})'
            )

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback,
        )
        send_future.add_done_callback(self._nav_goal_response_callback)

    def _nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Coverage chunk was rejected by Nav2')
            self._handle_chunk_failure('rejected')
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        now = self.get_clock().now()
        self._last_feedback_time = now
        self._update_movement_start()

        if self._chunk_distance_remaining is None:
            self._chunk_distance_remaining = feedback.distance_remaining
            self._last_progress_time = now
        elif self._chunk_distance_remaining - feedback.distance_remaining >= self._stall_progress_epsilon:
            self._chunk_distance_remaining = feedback.distance_remaining
            self._last_progress_time = now

        self._last_feedback_recoveries = feedback.number_of_recoveries

        poses_remaining = max(0, int(feedback.number_of_poses_remaining))
        current_index = max(self._next_index, self._active_chunk_end - poses_remaining)
        current_index = min(current_index, self._active_chunk_end)
        if current_index != self._active_feedback_goal_index:
            self._active_feedback_goal_index = current_index
            self._publish_current_goal(self._filtered_path[current_index])
            self._publish_markers()

    def _nav_result_callback(self, future):
        result = future.result()
        self._goal_handle = None

        if result is None:
            self._handle_chunk_failure('empty result')
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            completed_index = self._active_chunk_start
            was_escape_goal = self._active_is_escape
            if self._active_is_escape:
                self._pending_escape_goal = None
                self._next_index = max(0, self._pending_escape_resume_index)
                self.get_logger().info(
                    f'Escape reposition completed, resuming at waypoint index {self._next_index + 1}'
                )
            elif self._active_is_deferred:
                if self._deferred_indices and self._deferred_indices[0] == completed_index:
                    self._deferred_indices.popleft()
                    self._deferred_index_set.discard(completed_index)
            else:
                self._next_index = self._active_chunk_end + 1
                self._last_completed_index = max(self._last_completed_index, self._active_chunk_end)
            self._chunk_retry_count = 0
            self._active_chunk_start = -1
            self._active_chunk_end = -1
            self._active_is_deferred = False
            self._active_is_escape = False
            self._active_feedback_goal_index = self._next_index
            self._single_goal_mode = False
            self._retry_goal_override = None
            self._cancel_failure_reason = None
            self._publish_markers()
            if not was_escape_goal:
                self.get_logger().info(
                    f'Coverage chunk completed, next waypoint index {self._next_index + 1}'
                )
            return

        if result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Coverage chunk was canceled')
            reason = self._cancel_failure_reason or 'canceled'
            self._cancel_failure_reason = None
            self._handle_chunk_failure(reason)
            return

        self.get_logger().warn('Coverage chunk aborted by Nav2')
        self._cancel_failure_reason = None
        self._handle_chunk_failure('aborted')

    def _monitor_active_chunk(self):
        if self._goal_cancel_in_progress:
            return

        self._update_movement_start()

        now = self.get_clock().now()
        if self._last_progress_time is not None:
            stalled_for = (now - self._last_progress_time).nanoseconds / 1e9
            if stalled_for > self._stall_timeout_sec:
                self.get_logger().warn(
                    f'Coverage chunk stalled for {stalled_for:.1f}s, canceling and deferring current index'
                )
                self._cancel_active_goal('stalled')
                return

        if self._last_feedback_recoveries > self._max_recoveries_per_chunk:
            self.get_logger().warn(
                f'Coverage chunk exceeded {self._max_recoveries_per_chunk} recoveries, canceling'
            )
            self._cancel_active_goal('recoveries')

    def _cancel_active_goal(self, reason: str):
        if self._goal_handle is None:
            return
        self._cancel_failure_reason = reason
        self._goal_cancel_in_progress = True
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, _future):
        self._goal_cancel_in_progress = False

    def _handle_chunk_failure(self, reason: str):
        if self._active_is_escape:
            self._handle_escape_goal_failure(reason)
            return

        self._chunk_retry_count += 1
        failed_index = self._failed_index()

        if self._active_is_deferred:
            self._handle_deferred_goal_failure(reason, failed_index)
            return

        if reason in ('stalled', 'recoveries'):
            self._defer_index(failed_index, reason)
            return

        if self._chunk_retry_count <= self._max_single_goal_retries_before_defer:
            self._single_goal_mode = True
            self._next_index = failed_index
            self._retry_goal_override = self._build_retry_goal(failed_index)
            self._active_chunk_start = -1
            self._active_chunk_end = -1
            self._active_feedback_goal_index = self._next_index
            self._publish_markers()
            self.get_logger().warn(
                f'Retrying waypoint index {failed_index + 1} after {reason} '
                f'(single-goal retry {self._chunk_retry_count}/{self._max_single_goal_retries_before_defer})'
            )
            return

        self._defer_index(failed_index, reason)

    def _handle_deferred_goal_failure(self, reason: str, failed_index: int):
        if self._chunk_retry_count <= self._max_deferred_goal_retries:
            self._retry_goal_override = self._build_retry_goal(failed_index)
            self._active_chunk_start = -1
            self._active_chunk_end = -1
            self._active_feedback_goal_index = failed_index
            self._publish_markers()
            self.get_logger().warn(
                f'Retrying deferred waypoint index {failed_index + 1} after {reason} '
                f'({self._chunk_retry_count}/{self._max_deferred_goal_retries})'
            )
            return

        self._abandoned_indices.add(failed_index)
        if self._deferred_indices and self._deferred_indices[0] == failed_index:
            self._deferred_indices.popleft()
            self._deferred_index_set.discard(failed_index)
        self._chunk_retry_count = 0
        self._retry_goal_override = None
        self._active_chunk_start = -1
        self._active_chunk_end = -1
        self._active_is_deferred = False
        self.get_logger().warn(
            f'Giving up on deferred waypoint index {failed_index + 1} after repeated failures'
        )

    def _defer_index(self, failed_index: int, reason: str):
        if failed_index not in self._deferred_index_set and failed_index not in self._abandoned_indices:
            self._deferred_indices.append(failed_index)
            self._deferred_index_set.add(failed_index)
        self._next_index = max(self._next_index, failed_index + 1)
        self._chunk_retry_count = 0
        self._single_goal_mode = False
        self._retry_goal_override = None
        self._active_chunk_start = -1
        self._active_chunk_end = -1
        self._active_is_deferred = False
        self._active_is_escape = False
        self._active_feedback_goal_index = self._next_index
        self._pending_escape_goal = self._build_escape_goal(failed_index)
        self._pending_escape_resume_index = self._next_index
        self._publish_markers()
        if self._pending_escape_goal is not None:
            self.get_logger().warn(
                f'Deferred waypoint index {failed_index + 1} after {reason}; '
                f'will try an escape reposition before resuming at waypoint index {self._next_index + 1}'
            )
        else:
            self.get_logger().warn(
                f'Deferred waypoint index {failed_index + 1} after {reason}; '
                'continuing main path and will not revisit it before uncovered-area cleanup'
            )

    def _handle_escape_goal_failure(self, reason: str):
        self._pending_escape_goal = None
        self._chunk_retry_count = 0
        self._single_goal_mode = False
        self._retry_goal_override = None
        self._active_chunk_start = -1
        self._active_chunk_end = -1
        self._active_is_deferred = False
        self._active_is_escape = False
        self._active_feedback_goal_index = self._next_index
        self._publish_markers()
        self.get_logger().warn(
            f'Escape reposition failed after {reason}; continuing from current pose toward waypoint index {self._next_index + 1}'
        )

    def _finish_or_extend_coverage(self):
        coverage_ratio = self._estimate_coverage_ratio()
        if coverage_ratio is not None:
            self.get_logger().info(
                f'Coverage path exhausted at {coverage_ratio * 100.0:.1f}% estimated coverage'
            )

        skipped_deferred = 0
        while self._deferred_indices:
            deferred_index = self._deferred_indices.popleft()
            self._deferred_index_set.discard(deferred_index)
            if deferred_index not in self._abandoned_indices:
                skipped_deferred += 1
        if skipped_deferred > 0:
            self.get_logger().info(
                f'Skipping {skipped_deferred} deferred waypoint(s) and proceeding directly to uncovered-area cleanup'
            )

        if coverage_ratio is not None and coverage_ratio < self._desired_coverage_ratio:
            if (
                self._last_exhausted_coverage_ratio is not None
                and coverage_ratio <= self._last_exhausted_coverage_ratio + self._supplemental_progress_epsilon
            ):
                self._supplemental_cycles_without_progress += 1
            else:
                self._supplemental_cycles_without_progress = 0
            self._last_exhausted_coverage_ratio = coverage_ratio

            if self._supplemental_cycles_without_progress >= self._max_supplemental_cycles_without_progress:
                self.get_logger().warn(
                    'Coverage stayed below target and no measurable improvement was achieved over '
                    f'{self._max_supplemental_cycles_without_progress} supplemental cycles'
                )
            elif self._supplemental_pass_count >= self._max_supplemental_passes:
                self.get_logger().warn(
                    f'Coverage stayed below target, but maximum supplemental cleanup passes '
                    f'({self._max_supplemental_passes}) has been reached'
                )
            else:
                supplemental_goals = self._compute_supplemental_goals()
                if supplemental_goals:
                    previous_path_len = len(self._filtered_path)
                    self._supplemental_pass_count += 1
                    self._filtered_path.extend(self._orient_path(supplemental_goals))
                    self._next_index = max(self._next_index, previous_path_len)
                    self._publish_path(self._filtered_path_pub, self._filtered_path)
                    self._publish_markers()
                    self.get_logger().warn(
                        f'Coverage below target ({coverage_ratio * 100.0:.1f}% < '
                        f'{self._desired_coverage_ratio * 100.0:.1f}%), '
                        f'adding {len(supplemental_goals)} supplemental cleanup goals '
                        f'(pass {self._supplemental_pass_count}); resuming at index {self._next_index + 1}'
                    )
                    return

                self.get_logger().warn(
                    'Coverage path is exhausted and uncovered regions remain, '
                    'but no safe supplemental cleanup goals could be generated'
                )

        if coverage_ratio is not None and coverage_ratio >= self._desired_coverage_ratio:
            self.get_logger().info(
                f'Coverage threshold reached ({coverage_ratio * 100.0:.1f}% >= '
                f'{self._desired_coverage_ratio * 100.0:.1f}%)'
            )

        self._execution_complete = True
        self._publish_markers()
        elapsed_wall_time = self._elapsed_wall_time_string()
        if coverage_ratio is None:
            self.get_logger().info(
                f'Coverage execution complete (wall elapsed {elapsed_wall_time})'
            )
        else:
            self.get_logger().info(
                f'Coverage execution complete with estimated coverage {coverage_ratio * 100.0:.1f}% '
                f'(wall elapsed {elapsed_wall_time})'
            )

        if self._auto_dock_on_completion:
            self._try_send_dock_request()

    def _try_send_dock_request(self):
        if self._dock_request_sent:
            return

        if not self._dock_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info(f"Waiting for docking action server '{self._dock_action_name}'...")
            return

        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = self._dock_id
        goal.max_staging_time = float(self._dock_max_staging_time)
        goal.navigate_to_staging_pose = bool(self._dock_navigate_to_staging_pose)

        self._dock_request_sent = True
        self.get_logger().info(
            f"Coverage complete; sending dock request to '{self._dock_id}' "
            f"(navigate_to_staging_pose={goal.navigate_to_staging_pose})"
        )
        send_future = self._dock_client.send_goal_async(
            goal,
            feedback_callback=self._dock_feedback_callback,
        )
        send_future.add_done_callback(self._dock_goal_response_callback)

    def _dock_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Docking request was rejected')
            self._dock_complete = True
            return

        self._dock_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dock_result_callback)

    def _dock_feedback_callback(self, feedback_msg):
        state_names = {
            DockRobot.Feedback.NAV_TO_STAGING_POSE: 'navigating to staging pose',
            DockRobot.Feedback.INITIAL_PERCEPTION: 'detecting dock',
            DockRobot.Feedback.CONTROLLING: 'controlling into dock',
            DockRobot.Feedback.WAIT_FOR_CHARGE: 'waiting for charge',
            DockRobot.Feedback.RETRY: 'retrying',
        }
        feedback = feedback_msg.feedback
        state = state_names.get(feedback.state, f'state {feedback.state}')
        self.get_logger().debug(
            f'Docking feedback: {state}, retries={feedback.num_retries}'
        )

    def _dock_result_callback(self, future):
        result = future.result()
        self._dock_goal_handle = None
        self._dock_complete = True

        if result is None:
            self.get_logger().error('Docking finished with an empty result')
            return

        dock_result = result.result
        if result.status == GoalStatus.STATUS_SUCCEEDED and dock_result.success:
            self.get_logger().info(
                f"Docking complete at '{self._dock_id}' after coverage mission "
                f'(retries={dock_result.num_retries})'
            )
            return

        self.get_logger().error(
            f"Docking failed after coverage mission: status={result.status}, "
            f'error_code={dock_result.error_code}, message="{dock_result.error_msg}"'
        )

    def _select_start_index(self) -> int:
        if self._always_start_from_beginning:
            return 0

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None or not self._filtered_path:
            return 0

        nearest_index = 0
        nearest_distance = float('inf')
        for index, pose in enumerate(self._filtered_path):
            distance = self._distance_xy(
                robot_pose.pose.position.x,
                robot_pose.pose.position.y,
                pose.pose.position.x,
                pose.pose.position.y,
            )
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = index

        if nearest_distance <= self._start_near_robot_distance:
            return nearest_index
        return 0

    def _select_resume_index(self, robot_pose: Optional[PoseStamped]) -> int:
        if not self._filtered_path:
            return 0

        if robot_pose is None:
            return min(self._active_chunk_end + 1, len(self._filtered_path))

        search_start = min(max(self._next_index + 1, 0), len(self._filtered_path) - 1)
        best_index = min(search_start, len(self._filtered_path) - 1)
        best_distance = float('inf')

        for index in range(search_start, len(self._filtered_path)):
            pose = self._filtered_path[index]
            distance = self._distance_xy(
                robot_pose.pose.position.x,
                robot_pose.pose.position.y,
                pose.pose.position.x,
                pose.pose.position.y,
            )
            if distance < best_distance:
                best_distance = distance
                best_index = index
            if distance >= self._start_near_robot_distance:
                return index

        return best_index

    def _failed_index(self) -> int:
        if 0 <= self._active_feedback_goal_index < len(self._filtered_path):
            return self._active_feedback_goal_index
        if 0 <= self._active_chunk_start < len(self._filtered_path):
            return self._active_chunk_start
        return max(0, min(self._next_index, len(self._filtered_path) - 1))

    def _goal_for_index(self, index: int) -> PoseStamped:
        if self._retry_goal_override is not None and index == self._failed_index():
            return self._retry_goal_override
        return self._filtered_path[index]

    def _build_retry_goal(self, index: int) -> Optional[PoseStamped]:
        if not (0 <= index < len(self._filtered_path)):
            return None

        repaired = self._adjust_supplemental_goal(self._filtered_path[index])
        if repaired is None:
            return None
        return self._orient_pose_from_neighbors(index, repaired)

    def _build_escape_goal(self, failed_index: int) -> Optional[PoseStamped]:
        if not self._enable_escape_reposition:
            return None

        if not self._filtered_path:
            return None

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return None

        search_end = min(self._last_completed_index, failed_index - 1, len(self._filtered_path) - 1)
        if search_end < 0:
            return None
        search_start = max(0, search_end - max(1, self._escape_reposition_lookback_waypoints))

        best_index = -1
        best_pose = None
        best_score = float('-inf')
        for candidate_index in range(search_end, search_start - 1, -1):
            candidate_pose = self._filtered_path[candidate_index]
            if self._distance_between(robot_pose, candidate_pose) < self._escape_reposition_min_distance:
                continue

            adjusted_pose = self._adjust_supplemental_goal(candidate_pose)
            if adjusted_pose is None or not self._goal_is_traversable(adjusted_pose):
                continue

            clearance_score = self._clearance_score(
                adjusted_pose.pose.position.x,
                adjusted_pose.pose.position.y,
            )
            distance_score = self._distance_between(robot_pose, adjusted_pose)
            score = clearance_score * 4.0 - distance_score
            if score > best_score:
                best_score = score
                best_index = candidate_index
                best_pose = adjusted_pose

        if best_index < 0 or best_pose is None:
            return None
        return self._orient_pose_from_neighbors(best_index, best_pose)

    def _publish_path(self, publisher, poses: List[PoseStamped]):
        path = Path()
        path.header.frame_id = self._map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses
        publisher.publish(path)

    def _publish_current_goal(self, pose: PoseStamped):
        goal = PoseStamped()
        goal.header.frame_id = self._map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = pose.pose
        self._current_goal_pub.publish(goal)

    def _publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        line = Marker()
        line.header.frame_id = self._map_frame
        line.header.stamp = now
        line.ns = 'coverage_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.04
        line.color = ColorRGBA(r=0.07, g=0.76, b=0.92, a=0.95)
        line.points = [pose.pose.position for pose in self._filtered_path]
        marker_array.markers.append(line)

        waypoints = Marker()
        waypoints.header.frame_id = self._map_frame
        waypoints.header.stamp = now
        waypoints.ns = 'coverage_waypoints'
        waypoints.id = 1
        waypoints.type = Marker.SPHERE_LIST
        waypoints.action = Marker.ADD
        waypoints.scale.x = 0.09
        waypoints.scale.y = 0.09
        waypoints.scale.z = 0.09
        waypoints.color = ColorRGBA(r=1.0, g=0.65, b=0.10, a=0.95)
        waypoints.points = [pose.pose.position for pose in self._filtered_path]
        marker_array.markers.append(waypoints)

        current_goal = Marker()
        current_goal.header.frame_id = self._map_frame
        current_goal.header.stamp = now
        current_goal.ns = 'coverage_current_goal'
        current_goal.id = 2
        current_goal.type = Marker.SPHERE
        current_goal.action = Marker.ADD
        current_goal.scale.x = 0.18
        current_goal.scale.y = 0.18
        current_goal.scale.z = 0.18
        current_goal.color = ColorRGBA(r=0.20, g=0.95, b=0.30, a=0.95)
        active_index = self._active_feedback_goal_index
        if self._execution_complete and self._filtered_path:
            active_index = len(self._filtered_path) - 1
        if 0 <= active_index < len(self._filtered_path):
            current_goal.pose = self._filtered_path[active_index].pose
        else:
            current_goal.action = Marker.DELETE
        marker_array.markers.append(current_goal)

        self._markers_pub.publish(marker_array)

    def _estimate_coverage_ratio(self) -> Optional[float]:
        if self._progress_msg is None:
            return None

        reachable_cells = 0
        covered_cells = 0
        for progress_value in self._progress_msg.data:
            if progress_value >= 0:
                reachable_cells += 1
                if progress_value > 0:
                    covered_cells += 1

        if reachable_cells == 0:
            return None
        return covered_cells / reachable_cells

    def _compute_supplemental_goals(self) -> List[PoseStamped]:
        if self._map_msg is None or self._progress_msg is None:
            return []

        info = self._map_msg.info
        width = info.width
        height = info.height
        visited = [False] * (width * height)
        clusters = []

        for cell_y in range(height):
            row_offset = cell_y * width
            for cell_x in range(width):
                index = row_offset + cell_x
                if visited[index] or not self._is_uncovered_free_cell(index):
                    continue

                queue = deque([(cell_x, cell_y)])
                visited[index] = True
                cluster = []

                while queue:
                    current_x, current_y = queue.popleft()
                    cluster.append((current_x, current_y))

                    for step_x, step_y in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        next_x = current_x + step_x
                        next_y = current_y + step_y
                        if not (0 <= next_x < width and 0 <= next_y < height):
                            continue
                        next_index = next_y * width + next_x
                        if visited[next_index] or not self._is_uncovered_free_cell(next_index):
                            continue
                        visited[next_index] = True
                        queue.append((next_x, next_y))

                if len(cluster) >= self._min_uncovered_region_cells:
                    clusters.append(cluster)

        if not clusters:
            return []

        robot_pose = self._lookup_robot_pose()
        candidates = []
        for cluster in clusters:
            goals = self._cluster_to_sweep_goals(cluster, robot_pose)
            if not goals:
                fallback_goal = self._cluster_to_goal(cluster)
                goals = [fallback_goal] if fallback_goal is not None else []
            if not goals:
                continue
            distance = 0.0
            if robot_pose is not None:
                distance = self._distance_xy(
                    robot_pose.pose.position.x,
                    robot_pose.pose.position.y,
                    goals[0].pose.position.x,
                    goals[0].pose.position.y,
                )
            candidates.append((distance, len(cluster), goals))

        if not candidates:
            return []

        candidates.sort(key=lambda item: (-item[1], item[0]))
        selected = []
        for _, _, goals in candidates:
            remaining_budget = self._max_supplemental_goals_per_pass - len(selected)
            if remaining_budget <= 0:
                break
            selected.extend(goals[:remaining_budget])
        return selected

    def _is_uncovered_free_cell(self, index: int) -> bool:
        if self._map_msg is None or self._progress_msg is None:
            return False
        map_value = self._map_msg.data[index]
        progress_value = self._progress_msg.data[index]
        return 0 <= map_value < self._occupied_threshold and progress_value == OCC_GRID_FREE

    def _cluster_to_goal(self, cluster) -> Optional[PoseStamped]:
        if self._map_msg is None:
            return None

        center_x = sum(cell_x for cell_x, _ in cluster) / len(cluster)
        center_y = sum(cell_y for _, cell_y in cluster) / len(cluster)

        sorted_cells = sorted(
            cluster,
            key=lambda cell: (cell[0] - center_x) ** 2 + (cell[1] - center_y) ** 2,
        )

        for cell_x, cell_y in sorted_cells:
            adjusted = self._adjust_supplemental_goal(self._pose_from_cell(cell_x, cell_y))
            if adjusted is not None and self._goal_is_traversable(adjusted):
                return adjusted

        return None

    def _cluster_to_sweep_goals(
        self,
        cluster: List[Tuple[int, int]],
        robot_pose: Optional[PoseStamped],
    ) -> List[PoseStamped]:
        if self._map_msg is None or not cluster:
            return []

        info = self._map_msg.info
        cluster_set = set(cluster)
        x_values = [cell_x for cell_x, _ in cluster]
        y_values = [cell_y for _, cell_y in cluster]
        horizontal_span = max(x_values) - min(x_values) + 1
        vertical_span = max(y_values) - min(y_values) + 1
        sweep_horizontal = horizontal_span >= vertical_span

        spacing_cells = max(1, int(math.ceil(self._supplemental_sweep_spacing / info.resolution)))
        min_run_cells = max(2, int(math.ceil(self._supplemental_min_sweep_length / info.resolution)))
        runs = self._extract_cluster_sweep_runs(
            cluster_set,
            sweep_horizontal=sweep_horizontal,
            spacing_cells=spacing_cells,
            min_run_cells=min_run_cells,
        )
        if not runs:
            return []

        if robot_pose is not None:
            robot_cell_x = int((robot_pose.pose.position.x - info.origin.position.x) / info.resolution)
            robot_cell_y = int((robot_pose.pose.position.y - info.origin.position.y) / info.resolution)
            first_line = runs[0][0][1] if sweep_horizontal else runs[0][0][0]
            last_line = runs[-1][0][1] if sweep_horizontal else runs[-1][0][0]
            robot_line = robot_cell_y if sweep_horizontal else robot_cell_x
            if abs(last_line - robot_line) < abs(first_line - robot_line):
                runs.reverse()

        sweep_goals: List[PoseStamped] = []
        reverse_direction = False
        minimum_endpoint_spacing = max(self._min_waypoint_spacing, info.resolution * 2.0)

        for start_cell, end_cell in runs:
            start_pose = self._adjust_supplemental_goal(self._pose_from_cell(*start_cell))
            end_pose = self._adjust_supplemental_goal(self._pose_from_cell(*end_cell))
            if start_pose is None or end_pose is None:
                continue

            if self._distance_between(start_pose, end_pose) < minimum_endpoint_spacing:
                midpoint_pose = self._adjust_supplemental_goal(
                    self._pose_from_cell(
                        int(round((start_cell[0] + end_cell[0]) * 0.5)),
                        int(round((start_cell[1] + end_cell[1]) * 0.5)),
                    )
                )
                if midpoint_pose is not None:
                    self._append_pose_if_separated(sweep_goals, midpoint_pose, minimum_endpoint_spacing)
                continue

            ordered_start, ordered_end = (start_pose, end_pose)
            heading = self._segment_heading(start_pose, end_pose)
            if reverse_direction:
                ordered_start, ordered_end = end_pose, start_pose
                heading = self._normalize_angle(heading + math.pi)

            self._append_pose_if_separated(
                sweep_goals,
                self._copy_pose_with_heading(ordered_start, heading),
                minimum_endpoint_spacing,
            )
            self._append_pose_if_separated(
                sweep_goals,
                self._copy_pose_with_heading(ordered_end, heading),
                minimum_endpoint_spacing,
            )
            reverse_direction = not reverse_direction

        return sweep_goals

    def _extract_cluster_sweep_runs(
        self,
        cluster_cells,
        *,
        sweep_horizontal: bool,
        spacing_cells: int,
        min_run_cells: int,
    ) -> List[Tuple[Tuple[int, int], Tuple[int, int]]]:
        line_samples = {}
        for cell_x, cell_y in cluster_cells:
            line_key = cell_y if sweep_horizontal else cell_x
            coordinate = cell_x if sweep_horizontal else cell_y
            line_samples.setdefault(line_key, []).append(coordinate)

        selected_lines = self._select_scan_lines(sorted(line_samples.keys()), spacing_cells)
        runs: List[Tuple[Tuple[int, int], Tuple[int, int]]] = []
        for line_key in selected_lines:
            values = sorted(line_samples[line_key])
            for run_start, run_end in self._group_contiguous_values(values):
                if run_end - run_start + 1 < min_run_cells:
                    continue
                if sweep_horizontal:
                    runs.append(((run_start, line_key), (run_end, line_key)))
                else:
                    runs.append(((line_key, run_start), (line_key, run_end)))
        return runs

    @staticmethod
    def _select_scan_lines(line_keys: List[int], spacing_cells: int) -> List[int]:
        if not line_keys:
            return []

        selected = [line_keys[0]]
        last_selected = line_keys[0]
        for line_key in line_keys[1:]:
            if line_key - last_selected >= spacing_cells:
                selected.append(line_key)
                last_selected = line_key

        if selected[-1] != line_keys[-1] and line_keys[-1] - selected[-1] >= max(1, spacing_cells // 2):
            selected.append(line_keys[-1])
        return selected

    @staticmethod
    def _group_contiguous_values(values: List[int]) -> List[Tuple[int, int]]:
        if not values:
            return []

        runs = []
        run_start = values[0]
        previous = values[0]
        for value in values[1:]:
            if value != previous + 1:
                runs.append((run_start, previous))
                run_start = value
            previous = value
        runs.append((run_start, previous))
        return runs

    def _adjust_supplemental_goal(self, goal: PoseStamped) -> Optional[PoseStamped]:
        if self._goal_is_traversable(goal):
            return goal

        if self._map_msg is None:
            return goal

        info = self._map_msg.info
        step_cells = max(1, int(math.ceil(self._goal_adjustment_step / info.resolution)))
        max_radius_cells = max(1, int(math.ceil(self._goal_backoff_distance / info.resolution)))
        center_x = int((goal.pose.position.x - info.origin.position.x) / info.resolution)
        center_y = int((goal.pose.position.y - info.origin.position.y) / info.resolution)

        for radius in range(step_cells, max_radius_cells + 1, step_cells):
            for offset_y in range(-radius, radius + 1, step_cells):
                for offset_x in range(-radius, radius + 1, step_cells):
                    if max(abs(offset_x), abs(offset_y)) != radius:
                        continue
                    cell_x = center_x + offset_x
                    cell_y = center_y + offset_y
                    if not (0 <= cell_x < info.width and 0 <= cell_y < info.height):
                        continue

                    candidate = PoseStamped()
                    candidate.header.frame_id = self._map_frame
                    candidate.header.stamp = self.get_clock().now().to_msg()
                    candidate.pose.position.x = info.origin.position.x + (cell_x + 0.5) * info.resolution
                    candidate.pose.position.y = info.origin.position.y + (cell_y + 0.5) * info.resolution
                    candidate.pose.position.z = 0.0
                    candidate.pose.orientation = goal.pose.orientation

                    if self._goal_is_traversable(candidate):
                        return candidate

        return None

    def _pose_from_cell(self, cell_x: int, cell_y: int) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        if self._map_msg is not None:
            info = self._map_msg.info
            pose.pose.position.x = info.origin.position.x + (cell_x + 0.5) * info.resolution
            pose.pose.position.y = info.origin.position.y + (cell_y + 0.5) * info.resolution
        pose.pose.position.z = 0.0
        pose.pose.orientation = self._quaternion_from_yaw(0.0)
        return pose

    def _adjust_waypoint(self, index: int, waypoints: List[PoseStamped]) -> Optional[PoseStamped]:
        original = waypoints[index]
        if self._map_msg is None:
            return original

        directions = []
        if index > 0:
            directions.append(self._unit_direction(original, waypoints[index - 1]))
        if index < len(waypoints) - 1:
            directions.append(self._unit_direction(original, waypoints[index + 1]))

        if len(directions) == 2:
            blended = (directions[0][0] + directions[1][0], directions[0][1] + directions[1][1])
            norm = math.hypot(blended[0], blended[1])
            if norm > 1e-6:
                directions.insert(0, (blended[0] / norm, blended[1] / norm))

        if not directions:
            return original

        best_pose = original
        best_clearance = self._clearance_score(original.pose.position.x, original.pose.position.y)
        max_steps = max(1, int(math.ceil(self._goal_backoff_distance / max(self._goal_adjustment_step, 1e-3))))

        for step in range(max_steps + 1):
            distance = max(0.0, self._goal_backoff_distance - step * self._goal_adjustment_step)
            for direction_x, direction_y in directions:
                candidate = self._offset_pose(original, direction_x * distance, direction_y * distance)
                if not self._goal_is_traversable(candidate):
                    continue
                clearance = self._clearance_score(candidate.pose.position.x, candidate.pose.position.y)
                if clearance > best_clearance:
                    best_pose = candidate
                    best_clearance = clearance
            if best_pose is not original:
                break

        return best_pose

    def _goal_is_traversable(self, pose_stamped: PoseStamped) -> bool:
        if self._map_msg is None:
            return True

        info = self._map_msg.info
        map_x = int((pose_stamped.pose.position.x - info.origin.position.x) / info.resolution)
        map_y = int((pose_stamped.pose.position.y - info.origin.position.y) / info.resolution)

        if not (0 <= map_x < info.width and 0 <= map_y < info.height):
            return False

        clearance_cells = max(1, int(math.ceil(self._goal_clearance_radius / info.resolution)))
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
            for offset_x in range(-clearance_cells, clearance_cells + 1):
                cell_x = map_x + offset_x
                cell_y = map_y + offset_y
                if not (0 <= cell_x < info.width and 0 <= cell_y < info.height):
                    return -1.0
                value = self._map_msg.data[cell_y * info.width + cell_x]
                if value < 0 or value >= self._occupied_threshold:
                    best_distance = min(best_distance, math.hypot(offset_x, offset_y) * info.resolution)

        return best_distance if best_distance < float('inf') else clearance_cells * info.resolution

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

    def _orient_path(self, poses: List[PoseStamped]) -> List[PoseStamped]:
        if not poses:
            return []

        oriented = []
        for index, pose in enumerate(poses):
            oriented_pose = PoseStamped()
            oriented_pose.header.frame_id = self._map_frame
            oriented_pose.header.stamp = self.get_clock().now().to_msg()
            oriented_pose.pose.position = pose.pose.position

            if index < len(poses) - 1:
                heading = self._segment_heading(pose, poses[index + 1])
            elif index > 0:
                heading = self._segment_heading(poses[index - 1], pose)
            else:
                heading = 0.0

            oriented_pose.pose.orientation = self._quaternion_from_yaw(heading)
            oriented.append(oriented_pose)

        return oriented

    def _orient_pose_from_neighbors(self, index: int, pose: PoseStamped) -> PoseStamped:
        oriented_pose = PoseStamped()
        oriented_pose.header.frame_id = self._map_frame
        oriented_pose.header.stamp = self.get_clock().now().to_msg()
        oriented_pose.pose.position = pose.pose.position

        if 0 <= index < len(self._filtered_path) - 1:
            heading = self._segment_heading(pose, self._filtered_path[index + 1])
        elif index > 0:
            heading = self._segment_heading(self._filtered_path[index - 1], pose)
        else:
            heading = self._yaw_from_quaternion(pose.pose.orientation)

        oriented_pose.pose.orientation = self._quaternion_from_yaw(heading)
        return oriented_pose

    def _offset_pose(self, pose_stamped: PoseStamped, dx: float, dy: float) -> PoseStamped:
        shifted = PoseStamped()
        shifted.header.frame_id = self._map_frame
        shifted.header.stamp = self.get_clock().now().to_msg()
        shifted.pose.position.x = pose_stamped.pose.position.x + dx
        shifted.pose.position.y = pose_stamped.pose.position.y + dy
        shifted.pose.position.z = pose_stamped.pose.position.z
        shifted.pose.orientation = pose_stamped.pose.orientation
        return shifted

    def _copy_pose_with_heading(self, pose_stamped: PoseStamped, heading: float) -> PoseStamped:
        copied = PoseStamped()
        copied.header.frame_id = self._map_frame
        copied.header.stamp = self.get_clock().now().to_msg()
        copied.pose.position = pose_stamped.pose.position
        copied.pose.orientation = self._quaternion_from_yaw(heading)
        return copied

    def _append_pose_if_separated(
        self,
        poses: List[PoseStamped],
        pose: PoseStamped,
        minimum_spacing: float,
    ) -> None:
        if poses and self._distance_between(poses[-1], pose) < minimum_spacing:
            return
        poses.append(pose)

    def _occupancy_grid_to_image(self, map_msg: OccupancyGrid) -> Image:
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
    def _distance_between(pose_a: PoseStamped, pose_b: PoseStamped) -> float:
        return math.hypot(
            pose_b.pose.position.x - pose_a.pose.position.x,
            pose_b.pose.position.y - pose_a.pose.position.y,
        )

    @staticmethod
    def _point_to_segment_distance(point: PoseStamped, start_pose: PoseStamped, end_pose: PoseStamped) -> float:
        ax = start_pose.pose.position.x
        ay = start_pose.pose.position.y
        bx = end_pose.pose.position.x
        by = end_pose.pose.position.y
        px = point.pose.position.x
        py = point.pose.position.y

        abx = bx - ax
        aby = by - ay
        ab_len_sq = abx * abx + aby * aby
        if ab_len_sq < 1e-9:
            return math.hypot(px - ax, py - ay)

        t = ((px - ax) * abx + (py - ay) * aby) / ab_len_sq
        t = max(0.0, min(1.0, t))
        closest_x = ax + t * abx
        closest_y = ay + t * aby
        return math.hypot(px - closest_x, py - closest_y)

    @staticmethod
    def _distance_xy(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    @staticmethod
    def _segment_heading(start_pose: PoseStamped, end_pose: PoseStamped) -> float:
        return math.atan2(
            end_pose.pose.position.y - start_pose.pose.position.y,
            end_pose.pose.position.x - start_pose.pose.position.x,
        )

    @staticmethod
    def _unit_direction(start_pose: PoseStamped, end_pose: PoseStamped) -> Tuple[float, float]:
        dx = end_pose.pose.position.x - start_pose.pose.position.x
        dy = end_pose.pose.position.y - start_pose.pose.position.y
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return (0.0, 0.0)
        return (dx / norm, dy / norm)

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Quaternion:
        half = yaw * 0.5
        return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

    @staticmethod
    def _yaw_from_quaternion(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _elapsed_wall_time_string(self) -> str:
        if self._movement_start_wall_time is not None:
            elapsed_seconds = max(0.0, time.time() - self._movement_start_wall_time)
        elif self._launch_start_wall_time > 0.0:
            elapsed_seconds = max(0.0, time.time() - self._launch_start_wall_time)
        else:
            elapsed_seconds = max(0.0, time.monotonic() - self._node_wall_start_monotonic)
        return self._format_duration(elapsed_seconds)

    def _update_movement_start(self) -> None:
        if self._movement_start_wall_time is not None or self._movement_reference_pose is None:
            return

        current_pose = self._lookup_robot_pose()
        if current_pose is None:
            return

        moved_distance = self._distance_between(self._movement_reference_pose, current_pose)
        if moved_distance >= self._movement_start_distance_threshold:
            self._movement_start_wall_time = time.time()
            self.get_logger().info(
                f'Coverage elapsed timer started after robot moved {moved_distance:.2f} m'
            )

    @staticmethod
    def _format_duration(duration_seconds: float) -> str:
        total_seconds = int(round(duration_seconds))
        hours, remainder = divmod(total_seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        return f'{hours:02d}:{minutes:02d}:{seconds:02d}'


def main():
    rclpy.init()
    node = CoverageManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
