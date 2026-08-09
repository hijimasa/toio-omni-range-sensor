"""Tests for the wire protocol.

Runnable without ROS: `python3 -m pytest ros2/toio_omni_range/test`.

The interesting case is the bearing order. The wire order is not monotonic in
angle -- ray 36 is -180 deg -- and getting the reordering wrong produces a scan
that looks entirely plausible in RViz while being rotated by 180 degrees, which
is exactly the class of mistake the mirror calibration already cost a day to.
"""

import math

from toio_omni_range import omni_protocol as proto


def _scan(distance_cm, seq=1, version=proto.VERSION, vbat_mv=4100, uptime_s=42):
    return proto.Scan(version, seq, vbat_mv, uptime_s, distance_cm)


def test_packet_size_matches_the_firmware_struct():
    # 12 byte header + 72 u16. The firmware logs this number on boot.
    assert proto.PACKET_SIZE == 156


def test_round_trip():
    dist = [None if i % 3 else i * 0.5 for i in range(proto.N_RAYS)]
    got = proto.parse(proto.encode(_scan(dist, seq=1234)))
    assert got.seq == 1234
    assert got.vbat_mv == 4100
    assert got.uptime_s == 42
    assert got.distance_cm == dist


def test_no_echo_becomes_none():
    got = proto.parse(proto.encode(_scan([None] * proto.N_RAYS)))
    assert got.distance_cm == [None] * proto.N_RAYS


def test_rejects_foreign_and_short_payloads():
    good = proto.encode(_scan([1.0] * proto.N_RAYS))
    assert proto.parse(good[:-1]) is None
    assert proto.parse(b'\x00\x00' + good[2:]) is None      # wrong magic


def test_bearings_cover_the_circle_once():
    assert len(set(proto.BEARINGS_DEG)) == proto.N_RAYS
    assert min(proto.BEARINGS_DEG) == -180.0
    assert max(proto.BEARINGS_DEG) == 175.0
    assert proto.BEARINGS_DEG[0] == 0.0          # ray 0 is straight ahead


def test_bearing_order_is_monotonic_and_evenly_spaced():
    ordered = [proto.BEARINGS_DEG[i] for i in proto.BEARING_ORDER]
    assert ordered == sorted(ordered)
    steps = {round(b - a, 6) for a, b in zip(ordered, ordered[1:])}
    assert steps == {proto.RAY_STEP_DEG}
    # what the LaserScan header will claim, checked against the data
    angle_min = math.radians(ordered[0])
    increment = math.radians(proto.RAY_STEP_DEG)
    for k, bearing in enumerate(ordered):
        assert math.isclose(angle_min + k * increment, math.radians(bearing),
                            abs_tol=1e-9)


def test_bearing_order_puts_the_forward_ray_where_it_belongs():
    # ray 0 (0 deg) must land in the middle of the reordered array, not at index 0
    assert proto.BEARING_ORDER.index(0) == proto.N_RAYS // 2


def test_missed_scans_handles_the_u16_wrap():
    assert proto.missed_scans(None, 5) == 0
    assert proto.missed_scans(10, 11) == 0
    assert proto.missed_scans(10, 14) == 3
    assert proto.missed_scans(0xFFFF, 0) == 0
    assert proto.missed_scans(0xFFFE, 2) == 3


def test_is_unit():
    assert proto.is_unit('OmniRange-1A2B', [])
    assert proto.is_unit(None, [proto.SERVICE_UUID.upper()])
    assert not proto.is_unit('toio Core Cube-A1B', [])
    assert not proto.is_unit(None, [])


def test_commands_match_the_firmware_opcodes():
    assert proto.cmd_reboot() == b'\x01'
    assert proto.cmd_power_off() == b'\x02'
    assert proto.cmd_wifi_on() == b'\x04'
    assert proto.cmd_scan_period(200) == b'\x03\xc8\x00'
    # the firmware clamps; do it here too so the two agree on what was requested
    assert proto.cmd_scan_period(99999) == b'\x03\x10\x27'
