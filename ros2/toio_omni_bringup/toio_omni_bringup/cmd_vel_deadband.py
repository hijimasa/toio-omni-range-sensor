"""Last stage of the velocity chain: make small commands large enough to move.

    producers --> cmd_vel_raw --> collision_monitor --> cmd_vel_gated
                                                             |
                                                     cmd_vel_deadband
                                                             |
                                                          cmd_vel --> toio_ros2

Last, not first: the collision monitor's "approach" action scales the command
down as an obstacle nears, which would put it back under the deadband if the
compensation ran before it.

See motor_deadband.py for why this is needed at all. The short version is that
the cube ignores motor values below 8, and at these speeds -- 5 cm/s, a 2.7 cm
wheel base -- a planner's ordinary output lands under that, especially when
turning. Nothing reports an error; the robot simply does not move.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from toio_omni_bringup import motor_deadband as md


class CmdVelDeadband(Node):
    """Scale cmd_vel up until the cube's motors act on it."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_deadband')

        self.declare_parameter('min_motor', md.MIN_MOTOR)
        self.declare_parameter('zero_threshold', 0.5)
        self.declare_parameter('wheel_base', md.WHEEL_BASE)

        self.min_motor = self.get_parameter('min_motor').value
        self.zero_threshold = self.get_parameter('zero_threshold').value
        self.wheel_base = self.get_parameter('wheel_base').value

        self._boosted = 0
        self._passed = 0

        self.pub = self.create_publisher(Twist, 'cmd_vel', qos_profile=10)
        self.sub = self.create_subscription(Twist, 'cmd_vel_gated', self._on_cmd, 10)
        self.timer = self.create_timer(5.0, self._report)

        self.get_logger().info(
            'boosting commands below motor value %.0f (%.4f m/s straight, '
            '%.2f rad/s turning in place)'
            % (self.min_motor,
               self.min_motor / md.motor_units(1.0),
               self.min_motor / md.motor_units(1.0) / (self.wheel_base / 2.0)))

    def _on_cmd(self, msg: Twist) -> None:
        v, omega = md.scale_for_deadband(
            msg.linear.x, msg.angular.z, min_motor=self.min_motor,
            zero_threshold=self.zero_threshold, wheel_base=self.wheel_base)
        if (v, omega) != (msg.linear.x, msg.angular.z):
            self._boosted += 1
        else:
            self._passed += 1

        out = Twist()
        out.linear.x = v
        out.angular.z = omega
        self.pub.publish(out)

    def _report(self) -> None:
        total = self._boosted + self._passed
        if not total:
            return
        # Worth seeing: if almost everything is boosted, the planner's speed
        # limits are below what the hardware can do and should be raised
        # instead of leaning on this node.
        self.get_logger().info(
            '%d commands, %.0f%% boosted over the deadband'
            % (total, 100.0 * self._boosted / total),
            throttle_duration_sec=30.0)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDeadband()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
