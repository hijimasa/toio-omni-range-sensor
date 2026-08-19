#!/usr/bin/env python3
"""Step 4: fit radius -> distance from the recorded samples.

Model
-----
A ground point at horizontal distance d from the mirror axis is seen at
depression angle arctan(h/d) for a mirror h above the floor, and a convex
mirror maps that angle to an image radius that saturates at the horizon. So

    r(d) = a + b * arctan(d / c)

r rises steeply close in and flattens towards a + b*pi/2, which is the rim.
Inverting gives the distance the scanner actually needs:

    d(r) = c * tan((r - a) / b)

A plain polynomial would fit the samples just as well over the measured span
but goes the wrong way outside it -- it has no horizon, so it happily reports
finite distances for radii past the rim.

  ./fit_range.py                      fit and print the table
  ./fit_range.py --write              also store the fit in calib/range_fit.json
"""

import argparse
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from omni_common import CALIB_DIR, load_calib

SAMPLES = os.path.join(CALIB_DIR, "range_samples.json")
FIT = os.path.join(CALIB_DIR, "range_fit.json")


def model(p, d):
    a, b, c = p
    return a + b * np.arctan(d / max(c, 1e-6))


def invert(p, r):
    """Radius -> distance. NaN where the radius is past the modelled horizon."""
    a, b, c = p
    t = (np.asarray(r, float) - a) / b
    out = np.full(np.shape(t), np.nan, float)
    ok = (t > 0) & (t < np.pi / 2 * 0.999)
    out[ok] = c * np.tan(t[ok])
    return out


def fit(distances, radii):
    d = np.asarray(distances, float)
    r = np.asarray(radii, float)
    # a: radius extrapolated to d=0, b: span up to the horizon, c: the distance
    # scale at which the curve bends.
    p0 = [r.min() - 40.0, 120.0, max(d.mean(), 1.0)]
    res = least_squares(lambda p: model(p, d) - r, p0,
                        bounds=([-1e4, 1.0, 0.05], [1e4, 1e4, 1e3]))
    return res.x, res


def check_samples_frame(samples, cal, path):
    """Refuse samples that were taken at a different framesize than the calibration.

    record_sample() appends, so this file accumulates across rebuilds -- two
    mirrors and three framesizes have passed through it already. The radii are
    stored in calibration pixels, which makes samples from different framesizes
    look directly comparable, and the fit blends them into one curve without
    complaining. That is a plausible average of two geometries, and nothing
    downstream can tell. Some camera modes crop rather than scale, so the shared
    pixel scale is a fiction across a framesize change.
    """
    widths = sorted({int(x["frame_width"]) for x in samples if x.get("frame_width")})
    if not cal.frame_width:
        if len(widths) > 1:
            raise SystemExit("%s mixes samples from %s -- they cannot share one fit"
                             % (path, " and ".join("%dpx-wide frames" % w for w in widths)))
        return
    cal_w = int(cal.frame_width)
    stale = [w for w in widths if w != cal_w]
    if stale:
        raise SystemExit(
            "%s holds %d sample(s) taken at %s, but the calibration is for "
            "%dx%d frames. Different framesizes are not a rescaling of each "
            "other, so these cannot go in one fit.\n"
            "Move the old ones aside and re-record:\n"
            "    mv %s %s.bak"
            % (path, sum(1 for x in samples
                         if int(x.get("frame_width", 0)) in stale),
               " and ".join("%dpx wide" % w for w in stale),
               cal_w, int(cal.frame_height or cal_w), path, path))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--samples", default=SAMPLES)
    ap.add_argument("--write", action="store_true", help="save the fit to calib/")
    ap.add_argument("--out", default=FIT)
    ap.add_argument("--valid-range", type=float, nargs=2, metavar=("MIN", "MAX"),
                    help="distances (cm) to treat as usable. Defaults to the "
                         "measured span; widen it deliberately to accept "
                         "extrapolation, which is a judgement call about how "
                         "much accuracy the application needs")
    args = ap.parse_args()

    with open(args.samples) as f:
        data = json.load(f)
    s = data.get("samples", [])
    if len(s) < 3:
        raise SystemExit("need at least 3 samples to fit 3 parameters; have %d" % len(s))

    cal = load_calib()
    check_samples_frame(s, cal, args.samples)

    d = [x["distance_cm"] for x in s]
    r = [x["radius_px"] for x in s]
    p, res = fit(d, r)
    resid = model(p, np.array(d)) - np.array(r)
    rms = float(np.sqrt(np.mean(resid ** 2)))

    vmin, vmax = args.valid_range if args.valid_range else (min(d), max(d))
    print("samples : %d over %.1f-%.1f cm" % (len(s), min(d), max(d)))
    if args.valid_range:
        print("usable  : %.1f-%.1f cm (widened past the measured span on purpose)"
              % (vmin, vmax))
    print("fit     : r(d) = %.2f + %.2f * arctan(d / %.3f)" % tuple(p))
    print("residual: rms %.1f px, max %.1f px" % (rms, float(np.abs(resid).max())))
    if len(s) <= 4:
        print("WARNING : 3 free parameters and only %d samples -- the fit is barely"
              " constrained. Treat it as provisional until there are 6+." % len(s))
    print()
    print("  %-9s %-10s %s" % ("distance", "radius", "residual"))
    for x, y, e in zip(d, r, resid):
        print("  %6.2f cm %8.1f px %+8.1f" % (x, y, e))

    print()
    print("what the calibrated annulus covers:")
    for label, rad in (("r_in", cal.r_in), ("r_out (rim)", cal.r_out)):
        dd = invert(p, np.array([rad]))[0]
        print("  %-12s r=%6.1f px -> %s" % (label, rad,
              "past the horizon" if np.isnan(dd) else "%.1f cm" % dd))
    print()
    print("  %-8s %-10s %s" % ("radius", "distance", "cm per px"))
    for rad in np.linspace(cal.r_in, cal.r_out, 9):
        dd = invert(p, np.array([rad, rad + 1.0]))
        if np.isnan(dd[0]):
            print("  %6.1f  past the horizon" % rad)
        else:
            flag = ("" if min(d) <= dd[0] <= max(d)
                    else ("   <- extrapolated" if vmin <= dd[0] <= vmax
                          else "   <- outside the usable range"))
            print("  %6.1f  %7.2f cm  %.3f%s" % (rad, dd[0], dd[1] - dd[0], flag))

    # A tilt of the whole unit changes the depression angle of every ray by the
    # same amount, and r = a + b*(pi/2 - phi), so dr = -b * dphi.
    px_per_deg = p[1] * np.pi / 180.0
    print()
    print("tilt sensitivity: 1 deg of tilt moves the radius by %.1f px" % px_per_deg)
    print("  %-10s %s" % ("distance", "distance error per 1 deg of tilt"))
    for dist in (5.0, 7.0, 9.0, 11.0):
        r0 = model(p, np.array([dist]))[0]
        d2 = invert(p, np.array([r0 + px_per_deg]))[0]
        print("  %5.1f cm    %+.2f cm" % (dist, d2 - dist))

    if args.write:
        with open(args.out, "w") as f:
            json.dump(dict(model="r = a + b*arctan(d/c); d = c*tan((r-a)/b)",
                           a=float(p[0]), b=float(p[1]), c=float(p[2]),
                           units=dict(distance="cm", radius="calibration px"),
                           n_samples=len(s), rms_px=round(rms, 2),
                           measured_min_cm=float(min(d)), measured_max_cm=float(max(d)),
                           d_min=float(vmin), d_max=float(vmax),
                           # A fit is only meaningful for the geometry it was
                           # taken on. Changing framesize can change the crop,
                           # not just the scale, so record what it belongs to.
                           calib_r_in=float(cal.r_in), calib_r_out=float(cal.r_out)),
                      f, indent=2)
        print("\nwrote %s" % args.out)


if __name__ == "__main__":
    main()
