#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool


class CmdVelCameraSafetyFilter(Node):
    """Slow or stop forward Nav2 commands while a camera low obstacle is active."""

    def __init__(self):
        super().__init__('cmd_vel_camera_safety_filter')

        self.declare_parameter('enabled', True)
        self.declare_parameter('input_topic', 'cmd_vel_nav_raw')
        self.declare_parameter('output_topic', 'cmd_vel_camera_safe_raw')
        self.declare_parameter('detection_topic', '/low_obstacle/detected')
        self.declare_parameter('stamped', True)
        self.declare_parameter('detection_timeout', 0.6)
        self.declare_parameter('stop_forward_motion', True)
        self.declare_parameter('slow_forward_scale', 0.25)
        self.declare_parameter('allow_backward_motion', True)
        self.declare_parameter('max_forward_when_detected', 0.0)
        self.declare_parameter('angular_scale_when_detected', 0.7)
        self.declare_parameter('log_throttle_sec', 2.0)

        self._read_parameters()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        msg_type = TwistStamped if self._stamped else Twist
        self._publisher = self.create_publisher(msg_type, self._output_topic, 10)
        self.create_subscription(msg_type, self._input_topic, self._cmd_callback, 10)
        self.create_subscription(Bool, self._detection_topic, self._detection_callback, 10)

        self._last_detection_monotonic = 0.0
        self._detected = False
        self._last_log_monotonic = 0.0

        self.get_logger().info(
            f'CmdVelCameraSafetyFilter started: {self._input_topic} -> {self._output_topic}, '
            f'detection={self._detection_topic}'
        )

    def _read_parameters(self):
        self._enabled = bool(self.get_parameter('enabled').value)
        self._input_topic = str(self.get_parameter('input_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._detection_topic = str(self.get_parameter('detection_topic').value)
        self._stamped = bool(self.get_parameter('stamped').value)
        self._detection_timeout = max(0.05, float(self.get_parameter('detection_timeout').value))
        self._stop_forward_motion = bool(self.get_parameter('stop_forward_motion').value)
        self._slow_forward_scale = min(1.0, max(0.0, float(self.get_parameter('slow_forward_scale').value)))
        self._allow_backward_motion = bool(self.get_parameter('allow_backward_motion').value)
        self._max_forward_when_detected = max(0.0, float(self.get_parameter('max_forward_when_detected').value))
        self._angular_scale_when_detected = min(
            1.0,
            max(0.0, float(self.get_parameter('angular_scale_when_detected').value)),
        )
        self._log_throttle_sec = max(0.1, float(self.get_parameter('log_throttle_sec').value))

    def _on_set_parameters(self, _params):
        self._read_parameters()
        return SetParametersResult(successful=True)

    def _detection_callback(self, msg: Bool):
        self._detected = bool(msg.data)
        if self._detected:
            self._last_detection_monotonic = time.monotonic()

    def _cmd_callback(self, msg):
        if not self._enabled or not self._obstacle_active():
            self._publisher.publish(msg)
            return

        twist = msg.twist if self._stamped else msg
        if twist.linear.x > 0.0:
            if self._stop_forward_motion:
                twist.linear.x = min(twist.linear.x, self._max_forward_when_detected)
            else:
                twist.linear.x = min(
                    twist.linear.x * self._slow_forward_scale,
                    self._max_forward_when_detected,
                )

        if not self._allow_backward_motion and twist.linear.x < 0.0:
            twist.linear.x = 0.0

        twist.angular.z *= self._angular_scale_when_detected
        self._publisher.publish(msg)

        now = time.monotonic()
        if now - self._last_log_monotonic >= self._log_throttle_sec:
            self._last_log_monotonic = now
            self.get_logger().warn('Camera low-obstacle active; limiting Nav2 forward velocity')

    def _obstacle_active(self) -> bool:
        if self._detected:
            return True
        return (time.monotonic() - self._last_detection_monotonic) <= self._detection_timeout


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelCameraSafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
