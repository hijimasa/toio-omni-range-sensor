"""Drive a boustrophedon over the mat so the mapper sees all of it.

Goals go to toio_ros2's `cube_goal_pose`, which hands them to the cube's own
motor_control_target. The mat closes that loop in hardware, so there is nothing
here to tune and no navigation stack involved -- this runs during mapping, when
no map exists yet.

That command reaches the motors over BLE without passing through cmd_vel, which
means **the collision monitor cannot stop it**. Anything in the way gets pushed:
motor_control_target is a closed loop on the Position ID and will keep shoving
until it reaches the target or times out. So the watching has to happen here.
This node checks the scan ahead of itself, and when the way is blocked it takes
the motors back with a cmd_vel nudge -- a plain motor command overrides a target
motion -- reverses a little, and gives up on that waypoint.

The sensor is omnidirectional, so a waypoint does not have to be approached from
any particular heading and the robot never has to spin to look around. Coverage
is pure translation, which is why the pattern can be this simple.

The cube is stopped at each waypoint for `dwell_time` before moving on, and the
mapper only integrates while stopped. That is what keeps the BLE latency out of
the map instead of having to model it.

Arrival is detected by watching toio/pose: toio_ros2 logs the motor result but
does not publish it, so there is nothing to subscribe to.
"""

import math

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


SAVE_COMMAND = (
    'ros2 run nav2_map_server map_saver_cli '
    '-f src/toio-omni-range-sensor/ros2/toio_omni_bringup/map/arena '
    '--ros-args -p save_map_timeout:=5.0')


def wrap(angle):
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class CoverageDrive(Node):
    """Publish a lawnmower pattern of goals, watching the scan on the way."""

    def __init__(self) -> None:
        super().__init__('coverage_drive')

        # Mat extent in the map frame. toio_ros2 puts the near corner at the
        # origin and grows y downwards, so the mat is x in [0, W], y in [-H, 0].
        self.declare_parameter('min_x', 0.0)
        self.declare_parameter('max_x', 0.297)
        self.declare_parameter('min_y', -0.210)
        self.declare_parameter('max_y', 0.0)
        # Keep off the very edge: the cube's Position ID sensor stops reading
        # there, which is the same reason toio_ros2 clamps its own goals.
        self.declare_parameter('inset', 0.035)
        self.declare_parameter('row_spacing', 0.05)
        self.declare_parameter('dwell_time', 1.5)
        self.declare_parameter('goal_tolerance', 0.006)
        self.declare_parameter('goal_timeout', 25.0)
        # Settle time after the *first pose*, not after launch: the cube's BLE
        # scan can take 20 s, and a goal published before it connects is simply
        # lost -- the first waypoint then times out having never been driven.
        self.declare_parameter('start_delay', 2.0)
        # Keep the reverse nudge on the mat. Backing away from a wall in a
        # corner is how the cube ends up off the edge, where the Position ID
        # stops reading and the next goal fails with ERROR_ID_MISSED.
        self.declare_parameter('edge_margin', 0.015)
        # Obstacle handling. The distance is measured from the mirror axis, so
        # it has to clear both the blind zone (3.6 cm) and the cube's own half
        # width (1.6 cm); 5 cm leaves about 3.4 cm in front of the body.
        self.declare_parameter('stop_distance', 0.05)
        self.declare_parameter('stop_sector_deg', 50.0)
        self.declare_parameter('backoff_speed', 0.02)
        self.declare_parameter('backoff_time', 0.8)
        self.declare_parameter('turn_rate', 1.8)
        self.declare_parameter('edge_clearance', 0.01)
        # After a nudge, toio_ros2 sends one stop command when cmd_vel goes
        # quiet (its cmd_vel_timeout). Publishing the next goal before that
        # arrives would have the stop cancel the target motion instead. Waiting
        # it out is cheaper than reaching into the driver.
        self.declare_parameter('settle_time', 1.0)

        self.inset = self.get_parameter('inset').value
        self.dwell_time = self.get_parameter('dwell_time').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.stop_sector = math.radians(self.get_parameter('stop_sector_deg').value)
        self.backoff_speed = self.get_parameter('backoff_speed').value
        self.backoff_time = self.get_parameter('backoff_time').value
        self.turn_rate = self.get_parameter('turn_rate').value
        self.edge_clearance = self.get_parameter('edge_clearance').value
        self.settle_time = self.get_parameter('settle_time').value
        self.start_delay = self.get_parameter('start_delay').value
        self.edge_margin = self.get_parameter('edge_margin').value
        self.bounds = (self.get_parameter('min_x').value,
                       self.get_parameter('max_x').value,
                       self.get_parameter('min_y').value,
                       self.get_parameter('max_y').value)

        self.waypoints = self._plan()
        self._index = -1
        self._pose = None               # (x, y, yaw)
        self._scan = None
        self._sent_at = None
        self._arrived_at = None
        self._backoff_until = None
        self._resume_at = None
        self._nudge = None
        self._blocked = 0
        self._done = False

        self.goal_pub = self.create_publisher(
            PoseStamped, 'cube_goal_pose', qos_profile=10)
        # Into the deadband stage, not straight at the cube: 0.02 m/s is a
        # motor value of 7, which the cube ignores.
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_gated', qos_profile=10)
        self.pose_sub = self.create_subscription(
            PoseStamped, 'toio/pose', self._on_pose, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        # Late start: the cube has to be connected and localised on the mat
        # before the first goal means anything, and how long that takes depends
        # on a BLE scan.
        self._first_pose_at = None
        self.start_timer = self.create_timer(0.25, self._maybe_start)
        self.timer = None

        self.get_logger().info('%d waypoints planned' % len(self.waypoints))

    def _plan(self):
        # A waypoint closer to a wall than stop_distance is inside the abort
        # radius and can never be reached: the drive gives up on approach, every
        # time, having spent the timeout getting there. With walls at the mat
        # edge the row ends are exactly that, so the inset is raised to clear
        # the abort radius rather than letting the pattern ask for the
        # impossible. Coverage loses nothing real -- the sensor reaches 23 cm,
        # so the wall is seen from well back.
        inset = max(self.inset, self.stop_distance + self.edge_clearance)
        if inset > self.inset:
            self.get_logger().info(
                'inset raised %.3f -> %.3f m to clear the %.3f m abort radius'
                % (self.inset, inset, self.stop_distance))

        min_x = self.get_parameter('min_x').value + inset
        max_x = self.get_parameter('max_x').value - inset
        min_y = self.get_parameter('min_y').value + inset
        max_y = self.get_parameter('max_y').value - inset
        spacing = self.get_parameter('row_spacing').value

        if max_x <= min_x or max_y <= min_y:
            self.get_logger().error('the inset leaves no area to cover')
            return []

        n_rows = max(2, int(math.ceil((max_y - min_y) / spacing)) + 1)
        points = []
        for i in range(n_rows):
            y = min_y + (max_y - min_y) * i / (n_rows - 1)
            xs = [min_x, max_x] if i % 2 == 0 else [max_x, min_x]
            points.extend((x, y) for x in xs)
        return points

    def _maybe_start(self) -> None:
        if self._pose is None:
            # Loud, because "waiting" and "quietly broken" have looked the same
            # too many times in this stack.
            self.get_logger().warn(
                'waiting for toio/pose -- is the cube connected and on the mat?',
                throttle_duration_sec=5.0)
            return
        now = self.get_clock().now()
        if self._first_pose_at is None:
            self._first_pose_at = now
            self.get_logger().info('cube is on the mat; starting shortly')
            return
        if (now - self._first_pose_at).nanoseconds * 1e-9 < self.start_delay:
            return
        self.start_timer.cancel()
        self.timer = self.create_timer(0.1, self._tick)
        self._next_goal()

    def _backoff_stays_on_the_mat(self) -> bool:
        min_x, max_x, min_y, max_y = self.bounds
        x, y, yaw = self._pose
        distance = self.backoff_speed * self.backoff_time
        bx = x - distance * math.cos(yaw)
        by = y - distance * math.sin(yaw)
        return (min_x + self.edge_margin <= bx <= max_x - self.edge_margin and
                min_y + self.edge_margin <= by <= max_y - self.edge_margin)

    def _on_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _blocked_towards(self, bearing_body):
        """Report anything within stop_distance in the sector around this bearing."""
        scan = self._scan
        if scan is None or self._pose is None:
            return False
        for k, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > self.stop_distance:
                continue
            a = scan.angle_min + k * scan.angle_increment
            if abs(wrap(a - bearing_body)) <= self.stop_sector / 2.0:
                return True
        return False

    def _next_goal(self) -> None:
        self._index += 1
        if self._index >= len(self.waypoints):
            self._done = True
            # Into the mounted source tree, not the working directory: the
            # container's workspace root is not a mount and it runs with --rm,
            # so a map saved there is gone the moment you press Ctrl-C.
            self.get_logger().info(
                'coverage complete, %d waypoints abandoned as blocked. Save the '
                'map BEFORE leaving the container -- the workspace root is not '
                'mounted:\n    %s' % (self._blocked, SAVE_COMMAND))
            return

        x, y = self.waypoints[self._index]
        # Head towards the next waypoint so the cube does not turn on the spot
        # for nothing; with an omnidirectional sensor the heading is free.
        if self._index + 1 < len(self.waypoints):
            nx, ny = self.waypoints[self._index + 1]
            yaw = math.atan2(ny - y, nx - x)
        else:
            yaw = 0.0

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(msg)

        self._sent_at = self.get_clock().now()
        self._arrived_at = None
        self.get_logger().info('waypoint %d/%d -> (%.3f, %.3f)'
                               % (self._index + 1, len(self.waypoints), x, y))

    def _abandon(self, reason) -> None:
        """Take the motors back from the target motion and reverse a little."""
        self._blocked += 1
        self.get_logger().warn('waypoint %d abandoned: %s'
                               % (self._index + 1, reason))
        # Something has to be sent either way: a plain motor command is the only
        # thing that takes the target motion back off the cube, and skipping it
        # would leave motor_control_target shoving until it times out.
        self._nudge = Twist()
        if self._backoff_stays_on_the_mat():
            self._nudge.linear.x = -self.backoff_speed
            nudge_time = self.backoff_time
        else:
            # Against an edge there is no room to reverse, so turn instead --
            # the one nudge that cannot carry the cube off the mat.
            self.get_logger().warn('no room to reverse; turning instead')
            self._nudge.angular.z = self.turn_rate
            nudge_time = self.backoff_time / 2.0

        now = self.get_clock().now()
        self._backoff_until = now + rclpy.duration.Duration(seconds=nudge_time)
        self._resume_at = self._backoff_until + rclpy.duration.Duration(
            seconds=self.settle_time)
        self._sent_at = None

    def _tick(self) -> None:
        if self._done:
            return
        now = self.get_clock().now()

        if self._backoff_until is not None:
            if now < self._backoff_until:
                # A plain motor command overrides the running target motion,
                # which is the only way to interrupt it from outside the driver.
                self.cmd_pub.publish(self._nudge)
                return
            if now < self._resume_at:
                return          # let the driver's stop land before the next goal
            self._backoff_until = self._resume_at = self._nudge = None
            self._next_goal()
            return

        if self._pose is None or self._sent_at is None:
            return

        x, y = self.waypoints[self._index]
        dx, dy = x - self._pose[0], y - self._pose[1]
        distance = math.hypot(dx, dy)

        if self._arrived_at is None:
            if distance <= self.goal_tolerance:
                self._arrived_at = now
                return
            if self._blocked_towards(wrap(math.atan2(dy, dx) - self._pose[2])):
                self._abandon('something within %.0f mm ahead'
                              % (self.stop_distance * 1000.0))
            elif (now - self._sent_at).nanoseconds * 1e-9 > self.goal_timeout:
                # Not an error worth stopping for: the cube may have been
                # blocked by something the sensor cannot see.
                self._abandon('timed out %.1f mm away' % (distance * 1000.0))
            return

        if (now - self._arrived_at).nanoseconds * 1e-9 >= self.dwell_time:
            self._next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = CoverageDrive()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
