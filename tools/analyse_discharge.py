#!/usr/bin/env python3
"""Turn a discharge log from ble_client.py --battery into a drain figure.

  ./analyse_discharge.py calib/discharge_ble.csv
  ./analyse_discharge.py a.csv b.csv        # compare two configurations

A lithium cell's voltage-vs-charge curve is flat in the middle and steep at the
ends, so a slope in mV/h is only comparable between runs taken over the same
voltage band. The first minutes are dropped: right after the charger comes off
the surface charge decays and reads as a drain that is not there.
"""

import argparse
import csv
import sys


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append((float(r["t_s"]), int(r["vbat_mv"]), int(r["scans"])))
    return rows


def analyse(path, settle_s):
    rows = load(path)
    if len(rows) < 3:
        return None
    use = [r for r in rows if r[0] >= settle_s]
    if len(use) < 2:
        use = rows
    t0, v0, s0 = use[0]
    t1, v1, s1 = use[-1]
    span_h = (t1 - t0) / 3600.0
    if span_h <= 0:
        return None
    # Least squares, not endpoint-to-endpoint: the ADC reading bounces by tens
    # of millivolts between samples, so two endpoints give a slope dominated by
    # whichever way the noise happened to fall.
    n = len(use)
    mx = sum(r[0] for r in use) / n
    my = sum(r[1] for r in use) / n
    sxx = sum((r[0] - mx) ** 2 for r in use)
    sxy = sum((r[0] - mx) * (r[1] - my) for r in use)
    slope = (sxy / sxx) * 3600.0 if sxx else 0.0
    resid = [r[1] - (my + (sxy / sxx) * (r[0] - mx)) for r in use] if sxx else [0]
    rms = (sum(e * e for e in resid) / len(resid)) ** 0.5
    scan_hz = (s1 - s0) / (t1 - t0) if t1 > t0 else 0.0
    return dict(path=path, n=len(rows), span_h=span_h, v0=v0, v1=v1,
                slope=slope, scan_hz=scan_hz, rms=rms,
                total_h=(rows[-1][0] - rows[0][0]) / 3600.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--settle", type=float, default=300.0,
                    help="seconds to ignore at the start (default 300)")
    args = ap.parse_args()

    res = []
    for p in args.csv:
        r = analyse(p, args.settle)
        if r is None:
            print("%s: not enough samples" % p)
            continue
        res.append(r)
        print("%s" % p)
        print("  %d samples over %.2f h; fitted on the last %.2f h after settling"
              % (r["n"], r["total_h"], r["span_h"]))
        print("  %d -> %d mV   drain %.0f mV/h (fit residual %.0f mV rms)   scan %.1f Hz"
              % (r["v0"], r["v1"], -r["slope"], r["rms"], r["scan_hz"]))
        if r["slope"] < 0:
            # crude, but the useful question is "hours, not minutes?"
            print("  extrapolating this slope over 4.2->3.4 V gives %.1f h, but the"
                  % (800.0 / -r["slope"]))
            print("  cell's slope varies with charge state, so only trust that if the"
                  " run covered most of the band")
        else:
            print("  voltage did not fall: still on the charger, or the run was "
                  "too short to see past the surface-charge decay")

    if len(res) == 2 and res[0]["slope"] < 0 and res[1]["slope"] < 0:
        a, b = res
        print()
        print("%s vs %s: %.0f vs %.0f mV/h  ->  %.2fx"
              % (a["path"], b["path"], -a["slope"], -b["slope"],
                 a["slope"] / b["slope"]))
        print("NOTE: only comparable if both runs cover a similar voltage band; "
              "%d-%d mV vs %d-%d mV" % (a["v0"], a["v1"], b["v0"], b["v1"]))


if __name__ == "__main__":
    main()
