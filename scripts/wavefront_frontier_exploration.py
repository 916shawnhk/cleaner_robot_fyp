#!/usr/bin/env python3
# Wavefront Frontier Exploration node for cleaner_robot_fyp
# Adapted from nav2_wfd (nav2_wfe-main) by Samsung Research America
# Compatible with both simulation (use_sim_time=true) and real robot (use_sim_time=false)

import math
import sys
from enum import Enum

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                        QoSProfile, QoSReliabilityPolicy)

# ── Tunable constants ─────────────────────────────────────────────────────────
OCC_THRESHOLD = 10        # occupancy value above which a cell is "occupied"
MIN_FRONTIER_SIZE = 5     # minimum cells to consider a frontier valid


# ── Map helpers ───────────────────────────────────────────────────────────────

class OccupancyGrid2d:
    """Thin wrapper around nav_msgs/OccupancyGrid for cost/coord queries."""

    class CostValues(Enum):
        FreeSpace = 0
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map_msg: OccupancyGrid):
        self.map = map_msg

    def getCost(self, mx: int, my: int) -> int:
        return self.map.data[self._index(mx, my)]

    def getSizeX(self) -> int:
        return self.map.info.width

    def getSizeY(self) -> int:
        return self.map.info.height

    def mapToWorld(self, mx: int, my: int):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution
        return wx, wy

    def worldToMap(self, wx: float, wy: float):
        if wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y:
            raise ValueError('World coordinates out of map bounds')
        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        if my >= self.map.info.height or mx >= self.map.info.width:
            raise ValueError('Map coordinates out of bounds')
        return mx, my

    def _index(self, mx: int, my: int) -> int:
        return my * self.map.info.width + mx


# ── Frontier detection ────────────────────────────────────────────────────────

class PointClassification(Enum):
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8


class FrontierPoint:
    __slots__ = ('classification', 'mapX', 'mapY')

    def __init__(self, x: int, y: int):
        self.classification = 0
        self.mapX = x
        self.mapY = y


class FrontierCache:
    def __init__(self):
        self._cache: dict = {}

    def getPoint(self, x: int, y: int) -> FrontierPoint:
        key = (((x + y) * (x + y + 1)) // 2) + y
        if key not in self._cache:
            self._cache[key] = FrontierPoint(x, y)
        return self._cache[key]

    def clear(self):
        self._cache.clear()


def _get_neighbors(point: FrontierPoint, costmap: OccupancyGrid2d,
                   cache: FrontierCache):
    neighbors = []
    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if 0 < x < costmap.getSizeX() and 0 < y < costmap.getSizeY():
                neighbors.append(cache.getPoint(x, y))
    return neighbors


def _is_frontier_point(point: FrontierPoint, costmap: OccupancyGrid2d,
                        cache: FrontierCache) -> bool:
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False
    has_free = False
    for n in _get_neighbors(point, costmap, cache):
        cost = costmap.getCost(n.mapX, n.mapY)
        if cost > OCC_THRESHOLD:
            return False
        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            has_free = True
    return has_free


def _find_free(mx: int, my: int, costmap: OccupancyGrid2d, cache: FrontierCache):
    bfs = [cache.getPoint(mx, my)]
    while bfs:
        loc = bfs.pop(0)
        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
            return loc.mapX, loc.mapY
        for n in _get_neighbors(loc, costmap, cache):
            if not (n.classification & PointClassification.MapClosed.value):
                n.classification |= PointClassification.MapClosed.value
                bfs.append(n)
    return mx, my


def get_frontiers(pose, costmap: OccupancyGrid2d, logger) -> list:
    """
    Return list of (wx, wy) centroid coordinates for each discovered frontier.
    pose: geometry_msgs/Pose (position.x, position.y used)
    """
    cache = FrontierCache()

    try:
        mx, my = costmap.worldToMap(pose.position.x, pose.position.y)
    except ValueError as e:
        logger.warn(f'worldToMap failed: {e}')
        return []

    free_x, free_y = _find_free(mx, my, costmap, cache)
    start = cache.getPoint(free_x, free_y)
    start.classification = PointClassification.MapOpen.value
    queue = [start]
    frontiers = []

    while queue:
        p = queue.pop(0)
        if p.classification & PointClassification.MapClosed.value:
            continue

        if _is_frontier_point(p, costmap, cache):
            p.classification |= PointClassification.FrontierOpen.value
            frontier_queue = [p]
            new_frontier = []

            while frontier_queue:
                q = frontier_queue.pop(0)
                if q.classification & (PointClassification.MapClosed.value |
                                       PointClassification.FrontierClosed.value):
                    continue
                if _is_frontier_point(q, costmap, cache):
                    new_frontier.append(q)
                    for w in _get_neighbors(q, costmap, cache):
                        if not (w.classification & (PointClassification.FrontierOpen.value |
                                                    PointClassification.FrontierClosed.value |
                                                    PointClassification.MapClosed.value)):
                            w.classification |= PointClassification.FrontierOpen.value
                            frontier_queue.append(w)
                q.classification |= PointClassification.FrontierClosed.value

            coords = []
            for fp in new_frontier:
                fp.classification |= PointClassification.MapClosed.value
                coords.append(costmap.mapToWorld(fp.mapX, fp.mapY))

            if len(new_frontier) >= MIN_FRONTIER_SIZE:
                arr = np.array(coords)
                cx = float(np.mean(arr[:, 0]))
                cy = float(np.mean(arr[:, 1]))
                frontiers.append((cx, cy))

        for v in _get_neighbors(p, costmap, cache):
            if not (v.classification & (PointClassification.MapOpen.value |
                                        PointClassification.MapClosed.value)):
                if any(costmap.getCost(n.mapX, n.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value
                       for n in _get_neighbors(v, costmap, cache)):
                    v.classification |= PointClassification.MapOpen.value
                    queue.append(v)

        p.classification |= PointClassification.MapClosed.value

    return frontiers


# ── Main exploration node ─────────────────────────────────────────────────────

class WavefrontFrontierExplorer(Node):
    """
    Autonomous mapping node using Wavefront Frontier Detection.

    Subscribes to /map (OccupancyGrid from slam_toolbox) and /odom.
    Sends NavigateToPose goals to nav2 to drive towards unexplored frontiers.
    Stops automatically when no frontiers remain (map complete).

    Parameters
    ----------
    use_sim_time : bool  (set via launch argument or ROS param)
    exploration_rate : float  seconds between frontier re-evaluations (default 1.0)
    goal_timeout : float  seconds before a navigation goal is considered stuck (default 30.0)
    """

    def __init__(self):
        super().__init__('wavefront_frontier_explorer')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('exploration_rate', 1.0)
        self.declare_parameter('goal_timeout', 30.0)

        self._exploration_rate = self.get_parameter('exploration_rate').value
        self._goal_timeout = self.get_parameter('goal_timeout').value

        use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(
            f'WavefrontFrontierExplorer started | use_sim_time={use_sim_time} | '
            f'exploration_rate={self._exploration_rate}s | goal_timeout={self._goal_timeout}s'
        )

        # ── State ─────────────────────────────────────────────────────────────
        self._costmap: OccupancyGrid2d | None = None
        self._current_pose = None
        self._goal_handle = None
        self._exploring = False
        self._map_received = False
        self._pose_received = False
        self._mapping_started = False
        self._mapping_completed = False
        self._mapping_start_ns = None

        # ── QoS ───────────────────────────────────────────────────────────────
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)

        # ── Nav2 action client ────────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Exploration timer ─────────────────────────────────────────────────
        self._timer = self.create_timer(self._exploration_rate, self._exploration_step)

        self.get_logger().info('Waiting for /map and /odom ...')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _map_callback(self, msg: OccupancyGrid):
        self._costmap = OccupancyGrid2d(msg)
        if not self._map_received:
            self._map_received = True
            self.get_logger().info('Initial map received.')
            self._start_mapping_timer_if_ready()

    def _odom_callback(self, msg: Odometry):
        self._current_pose = msg.pose.pose
        if not self._pose_received:
            self._pose_received = True
            self.get_logger().info('Initial odometry received.')
            self._start_mapping_timer_if_ready()

    # ── Exploration logic ──────────────────────────────────────────────────────

    def _start_mapping_timer_if_ready(self):
        if self._mapping_started or not self._map_received or not self._pose_received:
            return

        self._mapping_started = True
        self._mapping_start_ns = self.get_clock().now().nanoseconds
        self.get_logger().info('Mapping timer started.')

    def _format_elapsed(self) -> str:
        if self._mapping_start_ns is None:
            return '00:00'

        elapsed_sec = max(
            0,
            int((self.get_clock().now().nanoseconds - self._mapping_start_ns) / 1e9),
        )
        minutes, seconds = divmod(elapsed_sec, 60)
        hours, minutes = divmod(minutes, 60)
        if hours > 0:
            return f'{hours:02d}:{minutes:02d}:{seconds:02d}'
        return f'{minutes:02d}:{seconds:02d}'

    def _complete_mapping(self):
        if self._mapping_completed:
            return

        self._mapping_completed = True
        self._timer.cancel()
        self.get_logger().info(
            f'No more frontiers detected — autonomous mapping complete! '
            f'Total mapping time: {self._format_elapsed()}'
        )
        raise SystemExit(0)

    def _exploration_step(self):
        """Called periodically. Sends a new goal only when the robot is free."""
        if not self._map_received or not self._pose_received:
            return  # not ready yet

        if self._exploring:
            return  # navigation goal still active

        frontiers = get_frontiers(self._current_pose, self._costmap, self.get_logger())

        if not frontiers:
            self._complete_mapping()

        # Choose frontier: farthest from current position for maximum coverage
        best = max(
            frontiers,
            key=lambda f: math.hypot(
                f[0] - self._current_pose.position.x,
                f[1] - self._current_pose.position.y,
            ),
        )

        self.get_logger().info(
            f'Frontiers found: {len(frontiers)} | '
            f'Sending goal to ({best[0]:.2f}, {best[1]:.2f})'
        )
        self._send_goal(best[0], best[1])

    def _send_goal(self, wx: float, wy: float):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("'navigate_to_pose' action server not available yet.")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = wx
        goal_pose.pose.position.y = wy
        goal_pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self._exploring = True
        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by nav2.')
            self._exploring = False
            return
        self._goal_handle = handle
        self.get_logger().info('Goal accepted.')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        self._exploring = False
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'Goal result error: {e}')
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Goal aborted by nav2 — will try next frontier.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal cancelled.')
        else:
            self.get_logger().warn(f'Goal ended with status: {status}')

    def _feedback_callback(self, feedback_msg):
        # Uncomment below for verbose distance-remaining feedback:
        # fb = feedback_msg.feedback
        # self.get_logger().debug(
        #     f'Distance remaining: {fb.distance_remaining:.2f} m')
        pass

    def cancel_current_goal(self):
        """Utility: cancel in-flight nav2 goal (e.g. on shutdown)."""
        if self._goal_handle is not None:
            self.get_logger().info('Cancelling current navigation goal.')
            self._goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = WavefrontFrontierExplorer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt — shutting down explorer.')
    finally:
        node.cancel_current_goal()
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
