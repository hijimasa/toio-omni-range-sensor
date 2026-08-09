#!/usr/bin/env python3
"""Baseline for the power comparison: stream JPEG over WiFi and log the battery.

This reproduces the configuration the project started with -- the PC pulls the
MJPEG stream and does the detection -- so its drain can be compared against the
BLE-only one from ble_client.py --battery. Writes the same CSV columns, so
analyse_discharge.py reads both.

Run it right after the BLE run: a lithium cell's voltage slope depends on where
in the discharge curve it sits, so the two runs have to cover a similar band.

  ./measure_wifi_drain.py --ip 192.168.128.161 --seconds 1800 --out calib/discharge_wifi.csv
"""

import argparse
import json
import threading
import time
import urllib.request


def stream_forever(url, stop, counter):
    """Keep the MJPEG stream saturated -- that is the load being measured."""
    while not stop.is_set():
        try:
            r = urllib.request.urlopen(url, timeout=8)
            buf = b""
            while not stop.is_set():
                chunk = r.read(4096)
                if not chunk:
                    break
                buf += chunk
                while True:
                    i = buf.find(b"\xff\xd8")
                    j = buf.find(b"\xff\xd9", i + 2)
                    if i == -1 or j == -1:
                        break
                    counter[0] += 1
                    buf = buf[j + 2:]
                if len(buf) > 1 << 20:
                    buf = b""
            r.close()
        except Exception:
            time.sleep(1.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True)
    ap.add_argument("--seconds", type=float, default=1800.0)
    ap.add_argument("--period", type=float, default=60.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--framesize", type=int, default=None,
                    help="set before streaming, e.g. 10 for VGA")
    args = ap.parse_args()

    base = "http://%s" % args.ip
    if args.framesize is not None:
        urllib.request.urlopen("%s/ctrl?var=framesize&val=%d" % (base, args.framesize),
                               timeout=5).read()
        time.sleep(2)

    stop = threading.Event()
    frames = [0]
    th = threading.Thread(target=stream_forever,
                          args=("%s:81/stream" % base, stop, frames), daemon=True)
    th.start()

    t0 = time.time()
    first = None
    with open(args.out, "w") as f:
        f.write("t_s,uptime_s,vbat_mv,scans\n")
        while time.time() - t0 < args.seconds:
            time.sleep(args.period)
            try:
                with urllib.request.urlopen("%s/status" % base, timeout=8) as r:
                    st = json.load(r)
            except Exception as exc:
                print("status failed: %s" % exc)
                continue
            mv = int(round(st["vbat"] * 1000))
            el = time.time() - t0
            if first is None:
                first = (el, mv)
            dt = el - first[0]
            rate = ((mv - first[1]) / dt * 3600.0) if dt > 30 else float("nan")
            print("[%6.0fs] vbat %4d mV  frames %6d (%.1f fps)   %s"
                  % (el, mv, frames[0], frames[0] / el,
                     "%+.0f mV/h" % rate if rate == rate else "(settling)"))
            # uptime is not exposed over HTTP; the column exists for the shared
            # CSV format and is not used by the analysis
            f.write("%.1f,0,%d,%d\n" % (el, mv, frames[0]))
            f.flush()
    stop.set()
    print("done: %d frames in %.0f s" % (frames[0], time.time() - t0))


if __name__ == "__main__":
    main()
