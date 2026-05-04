#!/usr/bin/env python3

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class NavReadyGuard(Node):
    def __init__(self):
        super().__init__('nav_ready_guard')

        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_timeout_sec', 1.0)
        self.declare_parameter('require_localization', True)
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')

        self._target_frame = self.get_parameter('target_frame').value
        self._source_frame = self.get_parameter('source_frame').value
        self._global_frame = self.get_parameter('global_frame').value
        self._scan_topic = self.get_parameter('scan_topic').value
        self._scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)
        self._require_localization = bool(self.get_parameter('require_localization').value)
        self._amcl_pose_topic = self.get_parameter('amcl_pose_topic').value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_scan_time = None
        self._last_scan_frame = None
        self._have_localization = False

        self.create_subscription(LaserScan, self._scan_topic, self._scan_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._amcl_pose_topic,
            self._amcl_pose_callback,
            10,
        )
        self.create_timer(0.2, self._check_ready)

        if self._require_localization:
            self.get_logger().info(
                f'Waiting for TF {self._target_frame}->{self._source_frame}, '
                f'{self._global_frame}->{self._target_frame}, fresh {self._scan_topic}, '
                f'and a valid AMCL pose...'
            )
        else:
            self.get_logger().info(
                f'Waiting for TF {self._target_frame}->{self._source_frame} '
                f'and fresh {self._scan_topic} data...'
            )

    def _scan_callback(self, msg: LaserScan):
        self._last_scan_time = rclpy.time.Time.from_msg(msg.header.stamp)
        self._last_scan_frame = msg.header.frame_id

    def _amcl_pose_callback(self, _msg: PoseWithCovarianceStamped):
        self._have_localization = True

    def _check_ready(self):
        now = self.get_clock().now()

        try:
            self._tf_buffer.lookup_transform(
                self._target_frame,
                self._source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return

        if self._require_localization:
            if not self._have_localization:
                return
            try:
                self._tf_buffer.lookup_transform(
                    self._global_frame,
                    self._target_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException:
                return

        if self._last_scan_time is None:
            return

        age = (now - self._last_scan_time).nanoseconds / 1e9
        if math.isnan(age) or age > self._scan_timeout_sec:
            return

        if not self._last_scan_frame:
            return

        # AMCL drops scans when the scan timestamp is older than the TF cache.
        # Require that the latest scan can actually be transformed at its own timestamp
        # before unblocking Nav2 startup.
        try:
            self._tf_buffer.lookup_transform(
                self._target_frame,
                self._last_scan_frame,
                self._last_scan_time,
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return

        self.get_logger().info(
            f'Navigation prerequisites ready: TF {self._target_frame}->{self._source_frame} available, '
            f'latest scan age {age:.2f}s, latest {self._scan_topic} transformable at scan time'
        )
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = NavReadyGuard()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
