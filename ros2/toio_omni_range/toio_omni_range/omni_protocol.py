"""Wire protocol of the OmniRange unit -- one definition, two consumers.

`tools/ble_client.py` and the ROS 2 driver both decode the same notification, so
the layout lives here instead of in either of them. This file mirrors
`firmware/timercam_stream/src/ble_out.h`; the two are a pair and have to be
changed together.

It lives inside the ROS package rather than in `tools/` because the package has
to be installable on its own -- colcon copies only this subtree -- while the
tools always run from a checkout and can reach in.

Deliberately stdlib only: no numpy, no cv2, no bleak. The driver on the robot
has no reason to carry the calibration tools' dependencies, and the transport is
the caller's business.
"""

import struct
from collections import namedtuple

# BLE identity. The unit advertises the service UUID and a name derived from its
# MAC, e.g. 'OmniRange-1A2B'.
SERVICE_UUID = '6f6d6e69-0001-4a52-b000-000000000001'
SCAN_UUID = '6f6d6e69-0002-4a52-b000-000000000001'
NAME_PREFIX = 'OmniRange'

MAGIC = 0x4F52          # 'OR'
VERSION = 2             # the version this module was written against
N_RAYS = 72
NO_ECHO = 0xFFFF        # nothing found in that direction
RAY_STEP_DEG = 360.0 / N_RAYS

# magic, version, n_rays, seq, vbat_mv, uptime_s, then N_RAYS x u16 of mm
_HEADER = '<HBBHHI'
HEADER_SIZE = struct.calcsize(_HEADER)
PACKET_SIZE = HEADER_SIZE + 2 * N_RAYS      # 156 bytes

# Commands, written to the scan characteristic (see ble_out.h for why there is
# no separate control attribute).
CMD_REBOOT = 0x01
CMD_POWER_OFF = 0x02
CMD_SCAN_PERIOD = 0x03
CMD_WIFI_ON = 0x04
MAX_SCAN_PERIOD_MS = 10000      # the firmware clamps to this

Scan = namedtuple('Scan', 'version seq vbat_mv uptime_s distance_cm')
"""One measurement. `distance_cm` has N_RAYS entries, None where nothing was
found; entry i is the distance along `bearing_deg(i)`."""


def bearing_deg(i):
    """Bearing of ray i in the ROS frame (x forward, CCW positive).

    Wrapped to [-180, 180), which is *not* monotonic in i: ray 36 is -180 and
    the second half of the array comes before the first half in angle. Consumers
    that need increasing angles must reorder -- see `BEARING_ORDER`.
    """
    return (i * RAY_STEP_DEG + 180.0) % 360.0 - 180.0


BEARINGS_DEG = [bearing_deg(i) for i in range(N_RAYS)]

BEARING_ORDER = sorted(range(N_RAYS), key=lambda i: BEARINGS_DEG[i])
"""Ray indices sorted by increasing bearing, for formats that require it."""


def parse(data):
    """Decode a scan notification, or None if the payload is not one."""
    if len(data) < PACKET_SIZE:
        return None
    magic, version, n_rays, seq, vbat_mv, uptime_s = struct.unpack_from(_HEADER, data, 0)
    if magic != MAGIC or n_rays != N_RAYS:
        return None
    raw = struct.unpack_from('<%dH' % N_RAYS, data, HEADER_SIZE)
    # the field is named dist_mm on the wire and the firmware packs cm * 10
    return Scan(version, seq, vbat_mv, uptime_s,
                [None if mm == NO_ECHO else mm / 10.0 for mm in raw])


def encode(scan):
    """Build a packet from a Scan. Only used by the tests and fake units."""
    raw = [NO_ECHO if d is None else min(int(round(d * 10.0)), NO_ECHO - 1)
           for d in scan.distance_cm]
    return (struct.pack(_HEADER, MAGIC, scan.version, N_RAYS,
                        scan.seq, scan.vbat_mv, scan.uptime_s) +
            struct.pack('<%dH' % N_RAYS, *raw))


def missed_scans(prev_seq, seq):
    """Count the scans lost between two sequence numbers, which wrap at u16."""
    if prev_seq is None:
        return 0
    return ((seq - prev_seq) & 0xFFFF) - 1


def is_unit(name, service_uuids=()):
    """Report whether this advertisement belongs to an OmniRange unit.

    Matches the service UUID or the name, because a BlueZ cache that predates
    the service can hand back an advertisement with the name only.
    """
    if name and name.startswith(NAME_PREFIX):
        return True
    return SERVICE_UUID in [u.lower() for u in (service_uuids or ())]


def cmd_reboot():
    """Reboot the unit."""
    return bytes([CMD_REBOOT])


def cmd_power_off():
    """Release the LDO hold. Only works on battery; with USB the rail stays up."""
    return bytes([CMD_POWER_OFF])


def cmd_wifi_on():
    """Start WiFi, for the calibration tools."""
    return bytes([CMD_WIFI_ON])


def cmd_scan_period(ms):
    """Set the minimum spacing between scans. 0 means as fast as the sensor allows."""
    return bytes([CMD_SCAN_PERIOD]) + struct.pack('<H', min(int(ms), MAX_SCAN_PERIOD_MS))
