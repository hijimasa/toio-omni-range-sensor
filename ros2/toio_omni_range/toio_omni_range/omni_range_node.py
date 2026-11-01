"""Publish the OmniRange unit's BLE scans as sensor_msgs/LaserScan.

The unit measures on-device and notifies 72 distances per scan, which is a
LaserScan almost verbatim. Publishing it as one is the whole point of this node:
everything downstream (RViz, costmaps, scan matching) then works without knowing
anything about this sensor.

Two things differ from a rotating lidar and are worth knowing:

* A scan is a *snapshot*. All 72 rays come from one camera frame, so there is no
  within-scan motion skew and `time_increment` is 0.
* The scan reaches about 23 cm. `range_min`/`range_max` come from the same
  calibration as the firmware, generated into params/omni_range_calib.yaml by
  tools/gen_firmware_calib.py -- do not type them in by hand.
"""

import asyncio
import contextlib
import math
import threading
import time

from bleak import BleakClient, BleakScanner
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan
from toio_omni_range import omni_protocol as proto


class OmniRangeNode(Node):
    """BLE client for one OmniRange unit, publishing LaserScan."""

    def __init__(self) -> None:
        super().__init__('omni_range_node')

        # Connecting by address skips the discovery scan. That matters here
        # because the toio cube is on the same HCI adapter and scanning while
        # connected is what destabilises those links; the node logs the address
        # of whatever it finds so it can be pinned after the first run.
        self.declare_parameter('device_address', '')
        # BlueZ adapter, e.g. 'hci1'. Leave empty for the default. Putting the
        # cube and the sensor on separate dongles is the fallback if they fight.
        self.declare_parameter('ble_adapter', '')
        self.declare_parameter('frame_id', 'omni_link')
        # Overridden by the generated calibration params; these defaults are the
        # mirror horizon of the 320x320 configuration and exist only so the node
        # starts without a params file.
        self.declare_parameter('range_min', 0.036)
        self.declare_parameter('range_max', 0.233)
        # Minimum spacing between scans, sent to the unit on connect. 0 leaves
        # the firmware default (200 ms) alone.
        self.declare_parameter('scan_period_ms', 0)
        self.declare_parameter('connect_timeout', 20.0)
        self.declare_parameter('reconnect_interval', 3.0)
        # A BLE link can go quiet without dropping, exactly like the WiFi stream
        # this replaced, so silence is treated as a dead link.
        self.declare_parameter('scan_timeout', 2.0)
        self.declare_parameter('publish_battery', True)

        self.device_address = self.get_parameter('device_address').value
        self.ble_adapter = self.get_parameter('ble_adapter').value
        self.frame_id = self.get_parameter('frame_id').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_period_ms = self.get_parameter('scan_period_ms').value
        self.connect_timeout = self.get_parameter('connect_timeout').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.scan_timeout = self.get_parameter('scan_timeout').value

        # LaserScan wants ranges ordered by increasing angle, which the wire
        # order is not (ray 36 is -180 deg). Derive the permutation from the
        # protocol instead of hard-coding a rotation, so it survives a change of
        # ray count or bearing convention.
        self._order = proto.BEARING_ORDER
        self._angle_min = math.radians(proto.BEARINGS_DEG[self._order[0]])
        self._angle_increment = math.radians(proto.RAY_STEP_DEG)
        self._angle_max = self._angle_min + (proto.N_RAYS - 1) * self._angle_increment

        self._connected = False
        self._last_scan_t = None
        self._scan_time = 0.2       # replaced by the measured interval
        self._warned_version = False
        self._last_seq = None
        self._stats = {'scans': 0, 'dropped': 0, 'hits': 0, 'out_of_range': 0,
                       'vbat_mv': 0, 'uptime_s': 0, 'reconnects': 0}

        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile=10)
        self.battery_pub = None
        if self.get_parameter('publish_battery').value:
            self.battery_pub = self.create_publisher(
                BatteryState, 'omni/battery', qos_profile=10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', qos_profile=10)
        self.diag_timer = self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            'range %.3f - %.3f m, %d rays every %.1f deg'
            % (self.range_min, self.range_max, proto.N_RAYS, proto.RAY_STEP_DEG))

        self._stop = threading.Event()
        self._ble_task = None
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._start_loop, daemon=True)
        self.thread.start()
        self._ble_future = asyncio.run_coroutine_threadsafe(self._ble_loop(), self.loop)

    def _start_loop(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    # ------------------------------------------------------------------ BLE ---

    def _adapter_kwargs(self) -> dict:
        # Only the BlueZ backend accepts 'adapter'; passing it elsewhere raises.
        return {'adapter': self.ble_adapter} if self.ble_adapter else {}

    async def _discover(self):
        dev = await BleakScanner.find_device_by_filter(
            lambda d, ad: proto.is_unit(d.name or ad.local_name, ad.service_uuids),
            timeout=self.connect_timeout, **self._adapter_kwargs())
        if dev is None:
            raise RuntimeError('no OmniRange unit found by BLE scan')
        self.get_logger().info(
            'found %s (%s). Set the device_address parameter to this address to '
            'skip the scan on every reconnect.' % (dev.name, dev.address))
        return dev

    async def _session(self) -> None:
        target = self.device_address or await self._discover()
        # BleakClient's own `timeout` only bounds the implicit discovery it does
        # for a bare address; connect() itself waits forever on BlueZ. A unit
        # whose previous link was not torn down cleanly stops advertising until
        # supervision expires, and without this the node hangs there in silence
        # -- no scan, no error, and a collision monitor that stops the robot
        # because its source went quiet.
        client = BleakClient(target, timeout=self.connect_timeout,
                             **self._adapter_kwargs())
        await asyncio.wait_for(client.connect(), timeout=self.connect_timeout)
        try:
            await client.start_notify(proto.SCAN_UUID, self._on_notify)
            if self.scan_period_ms > 0:
                await client.write_gatt_char(
                    proto.SCAN_UUID, proto.cmd_scan_period(self.scan_period_ms),
                    response=False)
            self._last_scan_t = time.monotonic()
            self._connected = True
            self.get_logger().info('connected to %s' % client.address)
            while rclpy.ok() and not self._stop.is_set() and client.is_connected:
                await asyncio.sleep(0.25)
                if time.monotonic() - self._last_scan_t > self.scan_timeout:
                    raise RuntimeError('no scan for %.1f s' % self.scan_timeout)
        finally:
            # Shielded: on shutdown this runs while the task is being cancelled,
            # and an unshielded await would be cancelled straight away, leaving
            # the unit measuring into a dead link.
            with contextlib.suppress(Exception):
                await asyncio.wait_for(
                    asyncio.shield(client.disconnect()), timeout=5.0)

    async def _ble_loop(self) -> None:
        self._ble_task = asyncio.current_task()
        while rclpy.ok() and not self._stop.is_set():
            try:
                await self._session()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                if self._stop.is_set():
                    return
                self.get_logger().error(
                    '%s. retrying in %.1f s...' % (e, self.reconnect_interval))
            finally:
                if self._connected:
                    self._stats['reconnects'] += 1
                self._connected = False
            await asyncio.sleep(self.reconnect_interval)

    def _on_notify(self, _characteristic, data: bytearray) -> None:
        """Handle a scan notification (called on the asyncio loop thread)."""
        stamp = self.get_clock().now().to_msg()
        scan = proto.parse(data)
        if scan is None:
            self.get_logger().warn('unparsable packet of %d bytes' % len(data))
            return
        if scan.version != proto.VERSION and not self._warned_version:
            self._warned_version = True
            self.get_logger().warn(
                'unit speaks protocol v%d, this driver v%d. Reflash the unit or '
                'update omni_protocol.py -- they are a pair.'
                % (scan.version, proto.VERSION))

        now = time.monotonic()
        dt = now - self._last_scan_t if self._last_scan_t is not None else None
        self._last_scan_t = now
        # Ignore the connect-to-first-scan gap and any stall, so scan_time stays
        # a description of the sensor rather than of the link.
        if dt is not None and 0.005 < dt < 1.0:
            self._scan_time = 0.9 * self._scan_time + 0.1 * dt

        self._stats['scans'] += 1
        self._stats['dropped'] += max(0, proto.missed_scans(self._last_seq, scan.seq))
        self._last_seq = scan.seq
        self._stats['vbat_mv'] = scan.vbat_mv
        self._stats['uptime_s'] = scan.uptime_s

        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.angle_min = self._angle_min
        msg.angle_max = self._angle_max
        msg.angle_increment = self._angle_increment
        # One camera frame, so every ray is simultaneous: no within-scan skew.
        msg.time_increment = 0.0
        msg.scan_time = self._scan_time
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        # Readings outside the fitted range are passed through rather than
        # dropped; range_min/range_max already tell consumers what to trust, and
        # silently deleting a too-close reading is the dangerous direction.
        ranges = []
        hits = out_of_range = 0
        for i in self._order:
            cm = scan.distance_cm[i]
            if cm is None:
                ranges.append(float('inf'))
                continue
            hits += 1
            m = cm / 100.0
            if m < self.range_min or m > self.range_max:
                out_of_range += 1
            ranges.append(m)
        msg.ranges = ranges
        self._stats['hits'] = hits
        self._stats['out_of_range'] += out_of_range
        self.scan_pub.publish(msg)

        if self.battery_pub is not None:
            bat = BatteryState()
            bat.header.stamp = stamp
            bat.voltage = scan.vbat_mv / 1000.0
            bat.percentage = float('nan')       # no discharge curve fitted yet
            bat.present = True
            bat.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            bat.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            self.battery_pub.publish(bat)

    # ------------------------------------------------------------ diagnostics ---

    def _publish_diagnostics(self) -> None:
        st = DiagnosticStatus()
        st.name = 'omni_range: link'
        st.hardware_id = self.device_address or proto.NAME_PREFIX
        if not self._connected:
            st.level = DiagnosticStatus.ERROR
            st.message = 'disconnected'
        elif self._stats['scans'] == 0:
            st.level = DiagnosticStatus.WARN
            st.message = 'connected, no scan yet'
        else:
            st.level = DiagnosticStatus.OK
            st.message = 'ok'
        st.values = [
            KeyValue(key='scans', value=str(self._stats['scans'])),
            KeyValue(key='dropped', value=str(self._stats['dropped'])),
            KeyValue(key='rate_hz', value='%.1f' % (1.0 / self._scan_time)),
            KeyValue(key='hits_last_scan',
                     value='%d/%d' % (self._stats['hits'], proto.N_RAYS)),
            KeyValue(key='out_of_range', value=str(self._stats['out_of_range'])),
            KeyValue(key='vbat_v', value='%.2f' % (self._stats['vbat_mv'] / 1000.0)),
            KeyValue(key='unit_uptime_s', value=str(self._stats['uptime_s'])),
            KeyValue(key='reconnects', value=str(self._stats['reconnects'])),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [st]
        self.diag_pub.publish(msg)

    # --------------------------------------------------------------- lifecycle ---

    def destroy_node(self):
        # Disconnect before stopping the loop rather than after. The unit only
        # measures while a client is connected, so a clean disconnect is what
        # puts it back to idle; killing the loop first leaves the link to time
        # out and the unit scanning into the void until it does.
        self._stop.set()
        if self._ble_task is not None:
            self.loop.call_soon_threadsafe(self._ble_task.cancel)
            try:
                self._ble_future.result(timeout=3.0)
            except asyncio.CancelledError:
                pass        # the normal path -- CancelledError is a BaseException
            except Exception:
                pass        # timed out, or the link was already gone
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OmniRangeNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
