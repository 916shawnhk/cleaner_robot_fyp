#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelFrictionCompensator(Node):
    """Raise tiny nonzero velocity commands enough to overcome static friction."""

    def __init__(self):
        super().__init__('cmd_vel_friction_compensator')

        self.declare_parameter('input_topic', 'cmd_vel_nav_raw')
        self.declare_parameter('output_topic', 'cmd_vel_nav')
        self.declare_parameter('stamped', True)
        self.declare_parameter('min_linear_x', 0.08)
        self.declare_parameter('min_angular_z', 0.35)
        self.declare_parameter('command_epsilon', 0.01)
        self.declare_parameter('max_linear_x', 0.28)
        self.declare_parameter('max_angular_z', 2.4)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._stamped = bool(self.get_parameter('stamped').value)
        self._min_linear_x = abs(float(self.get_parameter('min_linear_x').value))
        self._min_angular_z = abs(float(self.get_parameter('min_angular_z').value))
        self._command_epsilon = abs(float(self.get_parameter('command_epsilon').value))
        self._max_linear_x = abs(float(self.get_parameter('max_linear_x').value))
        self._max_angular_z = abs(float(self.get_parameter('max_angular_z').value))

        msg_type = TwistStamped if self._stamped else Twist
        self._publisher = self.create_publisher(msg_type, output_topic, 10)
        self.create_subscription(msg_type, input_topic, self._callback, 10)

        self.get_logger().info(
            'CmdVelFrictionCompensator started: '
            f'{input_topic} -> {output_topic}, '
            f'min_linear_x={self._min_linear_x:.3f}, '
            f'min_angular_z={self._min_angular_z:.3f}'
        )

    def _callback(self, msg):
        twist = msg.twist if self._stamped else msg

        twist.linear.x = self._floor_nonzero(
            twist.linear.x,
            self._min_linear_x,
            self._max_linear_x,
        )
        twist.angular.z = self._floor_nonzero(
            twist.angular.z,
            self._min_angular_z,
            self._max_angular_z,
        )

        self._publisher.publish(msg)

    def _floor_nonzero(self, value: float, minimum: float, maximum: float) -> float:
        if abs(value) < self._command_epsilon:
            return 0.0

        floored = math.copysign(max(abs(value), minimum), value)
        return max(-maximum, min(maximum, floored))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelFrictionCompensator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
