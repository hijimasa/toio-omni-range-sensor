"""Give nav2 the odometry chain it expects, from the mat's absolute pose.

toio_ros2 publishes map -> center directly, which is the honest description: the
Position ID is absolute and does not drift, so there is no odom frame to speak
of. nav2 nonetheless wants map -> odom -> base, so this node supplies that shape
without inventing anything -- map -> odom is a fixed identity and odom -> center
carries the pose. The two chains describe the same thing.

Run toio_ros2 with remap_cube_tf:=true next to this, or `center` gets two
parents and TF takes whichever arrived last.

Two things this node exists to handle:

* Position ID notifications stop when the cube stops moving, so the transform
  would go stale and nav2 would time out waiting for it. The transform is
  therefore republished on a timer, not on arrival.
* DWB samples velocities around the current one, taken from the odometry twist.
  Differentiating a ~1 mm quantised position at notification rate is mostly
  noise, so the twist is low-passed and decays to zero when the pose stops
  changing.
"""

import math

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def yaw_of(q):
    """Yaw of a quaternion, assuming the planar case this robot lives in."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    """Fold an angle difference into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class ToioOdomBridge(Node):
    """Republish the cube's mat pose as odometry and TF."""

    def __init__(self) -> None:
        super().__init__('toio_odom_bridge')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'center')
        self.declare_parameter('publish_rate', 30.0)
        # Low-pass on the differentiated pose. Lower is smoother and later.
        self.declare_parameter('twist_alpha', 0.3)
        # No new pose for this long means the cube is not moving.
        self.declare_parameter('twist_timeout', 0.2)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.twist_alpha = self.get_parameter('twist_alpha').value
        self.twist_timeout = self.get_parameter('twist_timeout').value

        self._pose = None               # latest PoseStamped
        self._prev = None               # (x, y, yaw, t) of the previous pose
        self._last_pose_t = None
        self._vx = self._vy = self._wz = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile=10)
        self.pose_sub = self.create_subscription(
            PoseStamped, 'toio/pose', self._on_pose, 10)

        self._publish_map_to_odom()
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish)

        self.get_logger().info(
            '%s -> %s is identity; %s -> %s follows toio/pose at %.0f Hz'
            % (self.map_frame, self.odom_frame, self.odom_frame,
               self.base_frame, self.publish_rate))

    def _publish_map_to_odom(self) -> None:
        # Identity, and static: the mat pose has no drift for an odom frame to
        # absorb. It exists only because nav2 requires the link to be there.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)

    def _on_pose(self, msg: PoseStamped) -> None:
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = yaw_of(msg.pose.orientation)

        if self._prev is not None:
            px, py, pyaw, pt = self._prev
            dt = t - pt
            if dt > 1e-3:
                dx, dy = x - px, y - py
                c, s = math.cos(yaw), math.sin(yaw)
                a = self.twist_alpha
                self._vx += a * ((dx * c + dy * s) / dt - self._vx)
                self._vy += a * ((-dx * s + dy * c) / dt - self._vy)
                self._wz += a * (wrap(yaw - pyaw) / dt - self._wz)
                self._prev = (x, y, yaw, t)
        else:
            self._prev = (x, y, yaw, t)

        self._pose = msg
        self._last_pose_t = self.get_clock().now()

    def _publish(self) -> None:
        if self._pose is None:
            return

        now = self.get_clock().now()
        # The cube stops sending Position IDs when it stops moving, so silence
        # means stationary rather than unknown.
        if self._last_pose_t is not None and \
                (now - self._last_pose_t).nanoseconds * 1e-9 > self.twist_timeout:
            self._vx = self._vy = self._wz = 0.0

        # Stamped now, not with the pose's own stamp: the transform is still
        # true while the cube sits still, and nav2 needs it to look current.
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self._pose.pose.position.x
        t.transform.translation.y = self._pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self._pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._pose.pose.position.x
        odom.pose.pose.position.y = self._pose.pose.position.y
        odom.pose.pose.orientation = self._pose.pose.orientation
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._wz
        # Position ID quantises to about a millimetre; the rest is unmeasured.
        odom.pose.covariance[0] = odom.pose.covariance[7] = 1e-6
        odom.pose.covariance[35] = 1e-4
        odom.twist.covariance[0] = odom.twist.covariance[7] = 1e-3
        odom.twist.covariance[35] = 1e-2
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ToioOdomBridge()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
