"""Build an occupancy grid from the scan, using the mat's pose as truth.

This is mapping with known poses, not SLAM. The Position ID gives an absolute,
drift-free pose, so the hard parts of SLAM -- scan matching and loop closure --
are not merely unnecessary, they would make the result worse: a matcher fed 72
beams reaching 23 cm has little to work with, and whatever it decides is
a correction applied to a pose that was already right.

So each scan is raycast into a log-odds grid from where the robot actually was.
The geometry lives in grid.py; this file is the ROS plumbing around it.

Save the result with the standard tool once the drive is done, into the mounted
source tree -- the container's workspace root is not a mount and it runs with
--rm, so a map written to the working directory dies with the container:

    ros2 run nav2_map_server map_saver_cli \
        -f src/toio-omni-range-sensor/ros2/toio_omni_bringup/map/arena \
        --ros-args -p save_map_timeout:=5.0
"""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from toio_omni_mapping.grid import LogOddsGrid


class GtMapper(Node):
    """Occupancy grid from LaserScan plus a known pose."""

    def __init__(self) -> None:
        super().__init__('gt_mapper')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.005)
        # Bounds of the grid in the map frame. The launch file derives them from
        # the same mat parameters toio_ros2 is given, plus a margin for walls
        # standing at or just outside the mat edge.
        self.declare_parameter('origin_x', -0.05)
        self.declare_parameter('origin_y', -0.26)
        self.declare_parameter('width_m', 0.21)
        self.declare_parameter('height_m', 0.297)
        self.declare_parameter('p_occ', 0.7)
        self.declare_parameter('p_free', 0.4)
        self.declare_parameter('l_clamp', 4.0)
        self.declare_parameter('publish_period', 1.0)
        # A scan is a snapshot, but it arrives tens of milliseconds after the
        # camera frame was taken, and the pose looked up on arrival is not the
        # pose at capture. At 5 cm/s that is a few millimetres, which is a whole
        # cell. Integrating only while stopped removes the error instead of
        # modelling it -- and the coverage drive stops at every waypoint anyway.
        self.declare_parameter('stationary_only', True)
        # Stationary is judged by displacement over a window, not by a speed.
        # A Position ID unit is about 1.4 mm and notifications arrive every
        # 40 ms or so, so one unit of flicker differentiates to 0.035 m/s --
        # far above any sensible speed threshold, and every scan gets thrown
        # away as "moving" while the cube sits perfectly still.
        self.declare_parameter('stationary_radius', 0.003)
        self.declare_parameter('stationary_window', 0.3)
        # Position ID notifications stop when the cube stops, so silence means
        # stationary rather than unknown. Without this the last speed estimate
        # freezes at whatever it was and nothing is ever integrated again.
        self.declare_parameter('pose_timeout', 0.4)

        self.map_frame = self.get_parameter('map_frame').value
        self.stationary_only = self.get_parameter('stationary_only').value
        self.stationary_radius = self.get_parameter('stationary_radius').value
        self.stationary_window = self.get_parameter('stationary_window').value
        self.pose_timeout = self.get_parameter('pose_timeout').value

        self.grid = LogOddsGrid(
            origin_x=self.get_parameter('origin_x').value,
            origin_y=self.get_parameter('origin_y').value,
            width_m=self.get_parameter('width_m').value,
            height_m=self.get_parameter('height_m').value,
            resolution=self.get_parameter('resolution').value,
            p_occ=self.get_parameter('p_occ').value,
            p_free=self.get_parameter('p_free').value,
            l_clamp=self.get_parameter('l_clamp').value)

        self.tf_buffer = Buffer()
        # spin_thread so a lookup that waits can actually see new transforms;
        # without it the buffer is only filled by this node's own executor,
        # which is blocked inside the callback doing the waiting.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self._recent = []               # (t, x, y) over the last window
        self._last_pose_rx = None
        self._integrated = 0
        self._skipped_moving = 0
        self._skipped_no_tf = 0

        # Transient local so a late RViz or map_saver_cli still receives it.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', map_qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, 'toio/pose', self._on_pose, 10)
        self.timer = self.create_timer(
            self.get_parameter('publish_period').value, self._publish_map)

        self.get_logger().info(
            'grid %d x %d cells at %.3f m, origin [%.3f, %.3f]'
            % (self.grid.cols, self.grid.rows, self.grid.resolution,
               self.grid.origin_x, self.grid.origin_y))

    def _on_pose(self, msg: PoseStamped) -> None:
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self._last_pose_rx = self.get_clock().now()
        self._recent.append((t, msg.pose.position.x, msg.pose.position.y))
        self._recent = [p for p in self._recent if t - p[0] <= self.stationary_window]

    def _is_stationary(self) -> bool:
        if self._last_pose_rx is None:
            return False                # nothing has ever placed the robot
        age = (self.get_clock().now() - self._last_pose_rx).nanoseconds * 1e-9
        if age > self.pose_timeout:
            return True                 # the cube stopped, so the IDs stopped
        if len(self._recent) < 2:
            return False
        _, x, y = self._recent[-1]
        return all(math.hypot(px - x, py - y) <= self.stationary_radius
                   for _, px, py in self._recent)

    def _on_scan(self, scan: LaserScan) -> None:
        if self.stationary_only and not self._is_stationary():
            self._skipped_moving += 1
            return
        try:
            # The latest transform, not the one at the scan's stamp. The stamp
            # is when the packet arrived over BLE, tens of milliseconds after
            # the camera frame and typically *ahead* of the newest transform,
            # so asking for it is asking tf2 to extrapolate into the future.
            # Taking the latest is exact here precisely because this only runs
            # while the robot is stationary.
            when = (rclpy.time.Time() if self.stationary_only
                    else rclpy.time.Time.from_msg(scan.header.stamp))
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, scan.header.frame_id, when,
                timeout=Duration(seconds=0.0 if self.stationary_only else 0.1))
        except Exception as e:
            self._skipped_no_tf += 1
            self.get_logger().warn('no transform for this scan: %s' % e,
                                   throttle_duration_sec=5.0)
            return

        sx = tf.transform.translation.x
        sy = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        placed = False
        for k, r in enumerate(scan.ranges):
            if math.isnan(r) or r < scan.range_min:
                continue                    # inside the blind zone: says nothing
            hit = r <= scan.range_max
            placed |= self.grid.integrate_ray(
                sx, sy, yaw + scan.angle_min + k * scan.angle_increment,
                r if hit else scan.range_max, hit)
        if not placed:
            self.get_logger().warn('the sensor is outside the grid; widen the bounds',
                                   throttle_duration_sec=5.0)
            return
        self._integrated += 1

    def _publish_map(self) -> None:
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = self.grid.resolution
        grid.info.width = self.grid.cols
        grid.info.height = self.grid.rows
        grid.info.origin.position.x = self.grid.origin_x
        grid.info.origin.position.y = self.grid.origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = self.grid.occupancy().ravel().tolist()
        self.map_pub.publish(grid)

        self.get_logger().info(
            '%d scans integrated, %.1f%% of the grid known '
            '(%d skipped moving, %d without a transform)'
            % (self._integrated, 100.0 * self.grid.known_fraction(),
               self._skipped_moving, self._skipped_no_tf),
            throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = GtMapper()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
