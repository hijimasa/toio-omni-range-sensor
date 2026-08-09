#!/usr/bin/env python3
"""Receive the 72 measured distances over BLE.

This is the production path: the unit does the measurement and sends 156 bytes
per scan instead of a ~26 kB JPEG, so WiFi can be switched off entirely.

  ./ble_client.py                 # find the unit, subscribe, print scans
  ./ble_client.py --seconds 20 --csv out.csv

The packet layout is not defined here -- it lives in omni_protocol, shared with
the ROS 2 driver, so the two cannot drift apart.
"""

import argparse
import asyncio
import os
import sys
import time

from bleak import BleakClient, BleakScanner

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "ros2", "toio_omni_range"))
from toio_omni_range import omni_protocol as proto     # noqa: E402


async def find_unit(timeout):
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: proto.is_unit(d.name or ad.local_name, ad.service_uuids),
        timeout=timeout)
    if dev is None:
        raise SystemExit("no OmniRange unit found. Is it powered and advertising?")
    print("found %s (%s)" % (dev.name, dev.address))
    return dev


async def run(args):
    print("scanning for the unit ...")
    dev = await find_unit(args.timeout)

    csv = open(args.csv, "w") if args.csv else None
    if csv:
        csv.write("t,seq," + ",".join("d%+d" % round(b) for b in proto.BEARINGS_DEG) + "\n")

    t0 = time.time()
    quiet = bool(args.battery)
    stats = {"n": 0, "last_seq": None, "dropped": 0, "vbat": 0, "uptime": 0}

    def on_notify(_, data):
        scan = proto.parse(data)
        if scan is None:
            print("unparsable packet of %d bytes" % len(data))
            return
        if scan.version != proto.VERSION:
            print("warning: unit speaks protocol v%d, this client v%d"
                  % (scan.version, proto.VERSION))
        stats["n"] += 1
        stats["dropped"] += max(0, proto.missed_scans(stats["last_seq"], scan.seq))
        stats["last_seq"] = scan.seq
        hits = [(b, d) for b, d in zip(proto.BEARINGS_DEG, scan.distance_cm)
                if d is not None]
        el = time.time() - t0
        stats["vbat"] = scan.vbat_mv
        stats["uptime"] = scan.uptime_s
        if quiet:
            return
        print("[%6.2fs] seq %5d  %2d/%d hits  %s"
              % (el, scan.seq, len(hits), proto.N_RAYS,
                 "  ".join("%+.0f:%.1fcm" % (b, d) for b, d in hits[:6]) or "-"))
        if csv:
            csv.write("%.3f,%d," % (el, scan.seq) +
                      ",".join("" if d is None else "%.1f" % d
                               for d in scan.distance_cm) + "\n")

    async with BleakClient(dev, timeout=args.timeout) as client:
        print("connected; %d bytes per scan" % proto.PACKET_SIZE)
        await client.start_notify(proto.SCAN_UUID, on_notify)
        if args.battery:
            blog = open(args.battery, "w")
            blog.write("t_s,uptime_s,vbat_mv,scans\n")
            first = None
            t_end = time.time() + args.seconds
            while time.time() < t_end:
                await asyncio.sleep(args.battery_period)
                mv, up = stats["vbat"], stats["uptime"]
                if not mv:
                    continue
                el = time.time() - t0
                if first is None:
                    first = (el, mv)
                dt = el - first[0]
                rate = ((mv - first[1]) / dt * 3600.0) if dt > 30 else float("nan")
                print("[%6.0fs] uptime %5ds  vbat %4d mV  scans %5d   %s"
                      % (el, up, mv, stats["n"],
                         "%+.0f mV/h" % rate if rate == rate else "(settling)"))
                blog.write("%.1f,%d,%d,%d\n" % (el, up, mv, stats["n"]))
                blog.flush()
            blog.close()
        else:
            await asyncio.sleep(args.seconds)
        await client.stop_notify(proto.SCAN_UUID)

    if csv:
        csv.close()
    el = time.time() - t0
    print("\n%d scans in %.1fs = %.1f Hz, %d dropped (sequence gaps)"
          % (stats["n"], el, stats["n"] / el if el else 0, stats["dropped"]))
    print("payload rate: %.0f B/s  (streaming the same result as JPEG was ~200 kB/s)"
          % (stats["n"] * proto.PACKET_SIZE / el if el else 0))


async def send_cmd(args):
    """One-shot command; the unit may vanish mid-write, which is the point."""
    print("scanning for the unit ...")
    dev = await find_unit(args.timeout)
    payload = {
        "reboot": proto.cmd_reboot,
        "off": proto.cmd_power_off,
        "wifi-on": proto.cmd_wifi_on,
        "scan-period": lambda: proto.cmd_scan_period(args.scan_period),
    }[args.cmd]()
    async with BleakClient(dev, timeout=args.timeout) as client:
        try:
            await client.write_gatt_char(proto.SCAN_UUID, payload, response=False)
        except Exception as exc:
            # a unit that switched itself off cannot acknowledge the write
            print("write returned: %s" % exc)
    print("sent %s" % args.cmd)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--csv", help="also write every scan to this CSV")
    ap.add_argument("--battery", metavar="CSV",
                    help="log battery voltage while scanning, for a discharge run")
    ap.add_argument("--battery-period", type=float, default=60.0,
                    help="seconds between battery samples (default 60)")
    ap.add_argument("--cmd", choices=["reboot", "off", "scan-period", "wifi-on"],
                    help="send a command instead of receiving. 'off' releases "
                         "the LDO hold and only works on battery -- with USB "
                         "plugged in the rail stays up")
    ap.add_argument("--scan-period", type=int, default=200,
                    help="ms, for --cmd scan-period")
    args = ap.parse_args()
    asyncio.run(send_cmd(args) if args.cmd else run(args))


if __name__ == "__main__":
    main()
