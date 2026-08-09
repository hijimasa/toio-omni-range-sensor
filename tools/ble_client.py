#!/usr/bin/env python3
"""Receive the 72 measured distances over BLE.

This is the production path: the unit does the measurement and sends 148 bytes
per scan instead of a ~26 kB JPEG, so WiFi can be switched off entirely.

  ./ble_client.py                 # find the unit, subscribe, print scans
  ./ble_client.py --seconds 20 --csv out.csv
"""

import argparse
import asyncio
import struct
import sys
import time

from bleak import BleakClient, BleakScanner

SERVICE_UUID = "6f6d6e69-0001-4a52-b000-000000000001"
SCAN_UUID = "6f6d6e69-0002-4a52-b000-000000000001"
N_RAYS = 72
NO_ECHO = 0xFFFF
HEADER = "<HBBHHI"        # magic, version, n_rays, seq, vbat_mv, uptime_s


def parse(data):
    if len(data) < struct.calcsize(HEADER) + 2 * N_RAYS:
        return None
    magic, ver, n, seq, vbat_mv, uptime = struct.unpack_from(HEADER, data, 0)
    if magic != 0x4F52 or n != N_RAYS:
        return None
    raw = struct.unpack_from("<%dH" % N_RAYS, data, struct.calcsize(HEADER))
    # bearing of ray i is i*5 deg in the ROS frame, wrapped to (-180, 180]
    out = []
    for i, mm in enumerate(raw):
        b = (i * 360.0 / N_RAYS + 180.0) % 360.0 - 180.0
        out.append((b, None if mm == NO_ECHO else mm / 10.0))
    return seq, ver, vbat_mv, uptime, out


async def run(args):
    print("scanning for the unit ...")
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: SERVICE_UUID.lower() in [u.lower() for u in (ad.service_uuids or [])]
        or (d.name or "").startswith("OmniRange"),
        timeout=args.timeout)
    if dev is None:
        raise SystemExit("no OmniRange unit found. Is it powered and advertising?")
    print("found %s (%s)" % (dev.name, dev.address))

    csv = open(args.csv, "w") if args.csv else None
    if csv:
        csv.write("t,seq," + ",".join("d%+d" % int(i * 5 if i * 5 <= 180 else i * 5 - 360)
                                      for i in range(N_RAYS)) + "\n")

    t0 = time.time()
    quiet = bool(args.battery)
    stats = {"n": 0, "last_seq": None, "dropped": 0, "vbat": 0, "uptime": 0}

    def on_notify(_, data):
        p = parse(data)
        if p is None:
            print("unparsable packet of %d bytes" % len(data))
            return
        seq, ver, vbat_mv, uptime, rays = p
        stats["n"] += 1
        if stats["last_seq"] is not None:
            gap = (seq - stats["last_seq"]) & 0xFFFF
            if gap > 1:
                stats["dropped"] += gap - 1
        stats["last_seq"] = seq
        hits = [(b, d) for b, d in rays if d is not None]
        el = time.time() - t0
        stats["vbat"] = vbat_mv
        stats["uptime"] = uptime
        if quiet:
            return
        print("[%6.2fs] seq %5d  %2d/%d hits  %s"
              % (el, seq, len(hits), N_RAYS,
                 "  ".join("%+.0f:%.1fcm" % (b, d) for b, d in hits[:6]) or "-"))
        if csv:
            csv.write("%.3f,%d," % (el, seq) +
                      ",".join("" if d is None else "%.1f" % d for _, d in rays) + "\n")

    async with BleakClient(dev, timeout=args.timeout) as client:
        print("connected; %d bytes per scan" % (struct.calcsize(HEADER) + 2 * N_RAYS))
        await client.start_notify(SCAN_UUID, on_notify)
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
        await client.stop_notify(SCAN_UUID)

    if csv:
        csv.close()
    el = time.time() - t0
    print("\n%d scans in %.1fs = %.1f Hz, %d dropped (sequence gaps)"
          % (stats["n"], el, stats["n"] / el if el else 0, stats["dropped"]))
    print("payload rate: %.0f B/s  (streaming the same result as JPEG was ~200 kB/s)"
          % (stats["n"] * (struct.calcsize(HEADER) + 2 * N_RAYS) / el if el else 0))


async def send_cmd(args):
    """One-shot command; the unit may vanish mid-write, which is the point."""
    print("scanning for the unit ...")
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: SERVICE_UUID.lower() in [u.lower() for u in (ad.service_uuids or [])]
        or (d.name or "").startswith("OmniRange"), timeout=args.timeout)
    if dev is None:
        raise SystemExit("no OmniRange unit found")
    print("found %s (%s)" % (dev.name, dev.address))
    if args.cmd == "reboot":
        payload = bytes([0x01])
    elif args.cmd == "off":
        payload = bytes([0x02])
    elif args.cmd == "wifi-on":
        payload = bytes([0x04])
    else:
        payload = bytes([0x03]) + struct.pack("<H", int(args.scan_period))
    async with BleakClient(dev, timeout=args.timeout) as client:
        try:
            await client.write_gatt_char(SCAN_UUID, payload, response=False)
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
