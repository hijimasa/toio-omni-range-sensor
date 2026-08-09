#!/usr/bin/env python3
"""Step 3: find the ground -> object boundary in 72 directions.

For every 5 deg direction the panorama column is walked outward from the robot.
The floor is the dominant, nearly neutral surface, so the first sustained
departure from it is the nearest obstacle in that direction.

Two cues are combined, because saturation alone is not enough:

  saturation  a coloured object sits far above the floor's saturation. This is
              the strong cue -- measured 6-10x the floor on a blue target.
  value       a grey or dark object barely moves saturation at all, so a large
              brightness departure counts as well.

Pixels listed in the ignore mask (the camera's own reflection, the mount, any
cable) are skipped; see mask_editor.py. Regions ignored *before* the first
usable row simply move the start of the walk outward.

The output is the boundary radius in source-frame pixels, NOT metres -- turning
a radius into a distance is step 4 and needs the mirror geometry calibrated
against targets at known ranges.

  ./omni_range.py --url http://<ip>:81/stream          live, with overlays
  ./omni_range.py --source shot.png --once --json out.json
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from omni_common import (N_RAYS, PANO_H, PANO_W, RAY_STEP_DEG, DEFAULT_CALIB,
                         DEFAULT_MASK, add_source_args, annotate_panorama,
                         build_maps, col_to_bearing, first_frame, load_calib,
                         load_mask, open_source, unwrap, RangeFit, check_frame)

COLS_PER_RAY = PANO_W // N_RAYS
RANGE_SAMPLES = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "..", "calib", "range_samples.json")


def record_sample(path, distance_cm, bearing_deg, radius_px, frame_width,
                  calib_width=640.0):
    """Append one (known distance, measured radius) pair for the step 4 fit.

    The radius is stored in the calibration's own pixel scale so samples taken
    at different framesizes stay comparable.
    """
    r_calib = radius_px * calib_width / float(frame_width)
    samples = []
    if os.path.exists(path):
        with open(path) as f:
            samples = json.load(f).get("samples", [])
    added = dict(distance_cm=float(distance_cm),
                 bearing_deg=float(bearing_deg),
                 radius_px=round(float(r_calib), 2),
                 frame_width=int(frame_width))
    samples.append(added)
    samples.sort(key=lambda s: s["distance_cm"])
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        json.dump(dict(note="radius_px is in calibration (VGA 640-wide) pixels",
                       samples=samples), f, indent=2)
    # Return the row that was just added, not samples[-1]: the list is sorted
    # by distance, so the last element is usually some earlier sample.
    return added, len(samples)


class RangeScanner:
    """Turns one frame into 72 boundary radii."""

    def __init__(self, cal, mask=None, sat_gain=1.0, val_gain=1.0,
                 threshold=3.0, val_threshold=6.0, min_run=8, ref_frac=0.5,
                 confirm=4.0,
                 pano_size=(PANO_W, PANO_H)):
        # Fraction of the annulus, measured from the inner edge, used to model
        # "what the floor looks like". The outer rows see the wall, furniture
        # and ceiling past the edge of the floor, and letting those into the
        # reference inflates its spread until real objects stop standing out:
        # measured, the widest empty-floor excursion fell from 4.6 to 2.9 sigma
        # when the reference was restricted to the inner half.
        self.ref_frac = ref_frac
        self.cal = cal
        self.w, self.h = pano_size
        self.mask = np.ones((self.h, self.w), bool) if mask is None else mask
        self.sat_gain = sat_gain
        self.val_gain = val_gain
        self.threshold = threshold
        # Brightness is the weaker, noisier cue and needs its own, higher bar.
        # The robot's own shadow measured 3.0 sigma in value and 0.0 in
        # saturation, so a shared 4.0 threshold leaves only 1 sigma of margin
        # against calling a shadow an obstacle. Shadows must not be masked out
        # instead: they move with the robot, so a static mask would be wrong.
        self.val_threshold = val_threshold
        # A run must peak this high to be taken as-is; see _first_run.
        self.confirm = confirm
        self.min_run = min_run
        self._maps = None
        self._maps_for = None

    def _ensure_maps(self, frame_width):
        if self._maps_for != frame_width:
            cal = self.cal.scaled_to(frame_width)
            self._maps = build_maps(cal, self.w, self.h)
            self._maps_for = frame_width
            self._scaled = cal
        return self._maps

    def deviation(self, pano):
        """Per-pixel 'how unlike the floor is this', in robust sigmas."""
        hsv = cv2.cvtColor(pano, cv2.COLOR_BGR2HSV)
        S = hsv[:, :, 1].astype(np.float32)
        V = hsv[:, :, 2].astype(np.float32)

        ref = self.mask.copy()
        if self.ref_frac < 1.0:
            ref[int(ref.shape[0] * self.ref_frac):, :] = False
        if not ref.any():
            ref = self.mask
        if not ref.any():
            return np.zeros_like(S)
        s_med = float(np.median(S[ref]))
        v_med = float(np.median(V[ref]))
        # MAD, not std: the objects we are looking for are outliers and would
        # inflate a plain standard deviation into hiding themselves.
        s_mad = float(np.median(np.abs(S[ref] - s_med))) * 1.4826
        v_mad = float(np.median(np.abs(V[ref] - v_med))) * 1.4826
        s_mad = max(s_mad, 2.0)
        v_mad = max(v_mad, 2.0)

        # Saturation only counts upward: an object is more colourful than the
        # floor, while a floor shadow is less and must not read as an object.
        d_sat = np.maximum(S - s_med, 0.0) / s_mad * self.sat_gain
        d_val = np.abs(V - v_med) / v_mad * self.val_gain
        # Both cues are rescaled onto the same "1.0 == fires" axis so the caller
        # only tunes one threshold, while value keeps its stricter bar.
        dev = np.maximum(d_sat / max(self.threshold, 1e-6),
                         d_val / max(self.val_threshold, 1e-6)) * self.threshold
        # Blank only what the ignore mask excludes. NOT `ref` -- that is the
        # inner slice used to model the floor, and zeroing by it silently made
        # the outer half of the annulus undetectable (targets past ~7 cm simply
        # vanished).
        dev[~self.mask] = 0.0
        # Smooth along the radius only; neighbouring directions stay independent.
        return cv2.GaussianBlur(dev, (1, 7), 0)

    def scan(self, frame):
        maps = self._ensure_maps(frame.shape[1])
        pano = unwrap(frame, *maps)
        dev = self.deviation(pano)

        radii = np.full(N_RAYS, np.nan)
        rows = np.full(N_RAYS, -1)
        strength = np.zeros(N_RAYS)
        for i in range(N_RAYS):
            c0 = i * COLS_PER_RAY
            block = dev[:, c0:c0 + COLS_PER_RAY]
            valid = self.mask[:, c0:c0 + COLS_PER_RAY]
            # Median across the columns of this direction rejects single-column
            # speckle without blurring into the neighbouring direction.
            prof = np.median(block, axis=1)
            usable = valid.any(axis=1)
            prof = np.where(usable, prof, 0.0)

            row = self._first_run(prof, usable, self.min_run)
            if row is not None:
                rows[i] = row
                radii[i] = self._scaled.radius_of_row(row, self.h)
                strength[i] = float(prof[row:row + self.min_run].mean())
        return dict(pano=pano, dev=dev, rows=rows, radii=radii,
                    strength=strength, bearings=np.arange(N_RAYS) * RAY_STEP_DEG)

    def _first_run(self, prof, usable, min_run):
        """Nearest run of `min_run` samples over threshold -- but only trust a
        marginal one if nothing more convincing lies further out.

        Taking the literal first excursion latches onto whatever weak artefact
        happens to sit closest in. Measured case: a target moved 5 cm -> 8 cm
        kept reporting the same radius, because the edge of the robot's own
        shadow cleared the threshold at 3.2 sigma before the real target did.
        A run whose peak reaches `confirm` is taken immediately; a weaker one is
        only accepted if no confirmed run exists anywhere further out.
        """
        runs = []
        n = len(prof)
        hit = prof > self.threshold
        run = 0
        start = None
        for r in range(n):
            ok = usable[r] and hit[r]
            if ok:
                if run == 0:
                    start = r
                run += 1
            else:
                if run >= min_run:
                    runs.append((start, float(prof[start:start + run].max())))
                run = 0
                start = None
        if run >= min_run:
            runs.append((start, float(prof[start:start + run].max())))

        if not runs:
            return None
        for start, peak in runs:
            if peak >= self.confirm:
                return start
        return runs[0][0]


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def draw_pano(result, scanner):
    vis = result["pano"].copy()
    h = vis.shape[0]
    for i in range(N_RAYS):
        c0 = i * COLS_PER_RAY
        r = result["rows"][i]
        if r >= 0:
            cv2.line(vis, (c0, int(r)), (c0 + COLS_PER_RAY - 1, int(r)),
                     (0, 0, 255), 2)
    # grey out whatever the mask excludes
    grey = (~scanner.mask)[:, :, None] * np.array([60, 0, 60], np.uint8)
    vis = np.clip(vis.astype(np.int16) + grey.astype(np.int16), 0, 255).astype(np.uint8)
    return annotate_panorama(vis)


def draw_polar(frame, result, cal):
    """Boundary points drawn back onto the raw frame."""
    vis = frame.copy()
    c = cal.scaled_to(frame.shape[1])
    sign = -1.0 if c.ccw else 1.0
    pts = []
    for i in range(N_RAYS):
        rad = result["radii"][i]
        a = np.deg2rad(c.theta0 + sign * i * RAY_STEP_DEG)
        rr = c.r_out if np.isnan(rad) else rad
        p = (int(c.cx + rr * np.cos(a)), int(c.cy + rr * np.sin(a)))
        pts.append(p)
        cv2.circle(vis, p, 3, (0, 0, 255) if not np.isnan(rad) else (80, 80, 80), -1)
    cv2.polylines(vis, [np.array(pts, np.int32)], True, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.circle(vis, (int(c.cx), int(c.cy)), int(c.r_out), (0, 255, 0), 1, cv2.LINE_AA)
    return vis


def to_records(result, cal, fit=None, frame_width=None):
    out = []
    for i in range(N_RAYS):
        rad = result["radii"][i]
        rec = dict(index=i,
                   bearing_deg=round(float(wrap180(result["bearings"][i])), 1),
                   radius_px=None if np.isnan(rad) else round(float(rad), 1),
                   strength=round(float(result["strength"][i]), 2),
                   hit=bool(not np.isnan(rad)))
        if fit is not None and not np.isnan(rad):
            # the fit lives in calibration pixels, the scan in frame pixels
            r_cal = rad * 640.0 / float(frame_width or 640.0)
            dist = float(fit.distance_cm(np.array([r_cal]))[0])
            rec["distance_cm"] = None if np.isnan(dist) else round(dist, 2)
            rec["extrapolated"] = bool(fit.extrapolated(np.array([dist]))[0])
        out.append(rec)
    return out


def print_table(records):
    hits = [r for r in records if r["hit"]]
    print("%d/%d directions have an obstacle before the mirror rim" % (len(hits), N_RAYS))
    for r in records:
        if not r["hit"]:
            continue
        d = ""
        if "distance_cm" in r:
            d = ("  distance %s cm%s"
                 % ("--" if r["distance_cm"] is None else "%6.2f" % r["distance_cm"],
                    " (extrapolated)" if r.get("extrapolated") else ""))
        print("  %+7.1f deg  radius %6.1f px  strength %5.1f%s"
              % (r["bearing_deg"], r["radius_px"], r["strength"], d))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_source_args(ap)
    ap.add_argument("--calib", default=DEFAULT_CALIB)
    ap.add_argument("--mask", default=DEFAULT_MASK)
    ap.add_argument("--threshold", type=float, default=3.0,
                    help="robust sigmas of saturation above the floor that count "
                         "as an object")
    ap.add_argument("--val-threshold", type=float, default=6.0,
                    help="same for brightness; higher because the robot's own "
                         "shadow already reaches 3 sigma there")
    ap.add_argument("--confirm", type=float, default=4.0,
                    help="a nearer but weaker run is only believed if no run "
                         "peaking this high exists further out")
    ap.add_argument("--min-run", type=int, default=8,
                    help="consecutive radial samples required (rejects speckle)")
    ap.add_argument("--sat-gain", type=float, default=1.0)
    ap.add_argument("--val-gain", type=float, default=1.0)
    ap.add_argument("--set-framesize", type=int, default=None, metavar="N",
                    help="POST /ctrl?var=framesize first. VGA is NOT enough for "
                         "this measurement -- 12 (XGA 1024x768) detects what "
                         "UXGA does, VGA misses most of it")
    ap.add_argument("--once", action="store_true", help="scan one frame and exit")
    ap.add_argument("--json", help="write the scan to this JSON file")
    ap.add_argument("--record-distance", type=float, metavar="CM",
                    help="append this scan's radius as a (distance, radius) "
                         "sample for the step 4 radius->distance fit")
    ap.add_argument("--record-bearing", type=float, default=0.0, metavar="DEG",
                    help="which direction the known target sits in (default 0)")
    ap.add_argument("--record-bar", action="store_true",
                    help="the target is a straight bar perpendicular to the "
                         "forward axis, so --record-distance is its "
                         "perpendicular distance; the sample is stored as "
                         "distance/cos(bearing)")
    ap.add_argument("--record-file", default=RANGE_SAMPLES)
    args = ap.parse_args()

    cal = load_calib(args.calib)
    mask = load_mask(args.mask)
    fit = RangeFit.load()
    if fit is None:
        print("note: no calib/range_fit.json -- reporting radii only. Run fit_range.py.")
    else:
        ok = fit.matches(cal)
        if ok is False:
            print("WARNING: calib/range_fit.json was taken on r_in=%.0f r_out=%.0f but "
                  "the calibration in use is r_in=%.0f r_out=%.0f. Distances from it "
                  "would be wrong, so only radii are reported. Re-record samples and "
                  "re-run fit_range.py for this geometry."
                  % (fit.calib_r_in, fit.calib_r_out, cal.r_in, cal.r_out))
            fit = None
        elif ok is None:
            print("note: range_fit.json predates the calibration stamp; cannot check "
                  "it belongs to this geometry.")
    if mask.all():
        print("note: no ignore mask at %s -- the camera's own reflection will be "
              "read as an obstacle. Run mask_editor.py first." % args.mask)

    scanner = RangeScanner(cal, mask, args.sat_gain, args.val_gain,
                           args.threshold, args.val_threshold, args.min_run,
                           confirm=args.confirm)

    if args.set_framesize is not None:
        base = (args.url or args.still_url or "")
        host = base.split("/")[2].split(":")[0] if "//" in base else None
        if host:
            import urllib.request
            urllib.request.urlopen("http://%s/ctrl?var=framesize&val=%d"
                                   % (host, args.set_framesize), timeout=5).read()
            print("framesize -> %d" % args.set_framesize)
            # the driver serves a couple of stale buffers after a size change
            time.sleep(2.0)

    cap = open_source(args)
    frame = first_frame(cap)
    check_frame(cal, frame)

    if args.once:
        result = scanner.scan(frame)
        records = to_records(result, cal, fit, frame.shape[1])
        print_table(records)
        if args.record_distance is not None:
            i = int(round((args.record_bearing % 360.0) / RAY_STEP_DEG)) % N_RAYS
            rad = result["radii"][i]
            if np.isnan(rad):
                print("no detection at %+.1f deg -- nothing recorded"
                      % args.record_bearing)
            else:
                d = args.record_distance
                if args.record_bar:
                    d /= max(np.cos(np.deg2rad(args.record_bearing)), 1e-3)
                added, n = record_sample(args.record_file, d,
                                         args.record_bearing, rad, frame.shape[1])
                print("recorded %.2f cm -> radius %.1f px (calib scale); "
                      "%d sample(s) in %s"
                      % (added["distance_cm"], added["radius_px"],
                         n, args.record_file))
                # Compare against where the mask actually opens up in THIS
                # direction, not the global r_in: the reflection reaches
                # different radii in different directions.
                c0 = i * COLS_PER_RAY
                usable = mask[:, c0:c0 + COLS_PER_RAY].any(axis=1)
                edge_row = int(np.argmax(usable)) if usable.any() else 0
                edge_r = cal.radius_of_row(edge_row, PANO_H)
                if added["radius_px"] < edge_r + 8.0:
                    print("  WARNING: the mask opens at radius %.1f in this "
                          "direction and the hit is at %.1f -- that is the mask "
                          "boundary, not the target. Nothing closer than about "
                          "this radius can be measured here; move the target "
                          "further out." % (edge_r, added["radius_px"]))
        if args.json:
            with open(args.json, "w") as f:
                json.dump(dict(calib=cal.to_dict(), scan=records), f, indent=2)
            print("wrote %s" % args.json)
        cap.release()
        return

    win = "scan (q=quit c=capture)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.namedWindow("panorama", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("threshold x10", win, int(args.threshold * 10), 200,
                       lambda v: None)
    cv2.createTrackbar("min_run", win, args.min_run, 20, lambda v: None)

    t_prev, fps = time.time(), 0.0
    while True:
        new = cap.read()
        if new is not None:
            frame = new
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6)
            t_prev = now

        scanner.threshold = max(cv2.getTrackbarPos("threshold x10", win), 1) / 10.0
        scanner.min_run = max(cv2.getTrackbarPos("min_run", win), 1)

        result = scanner.scan(frame)
        n_hit = int(np.sum(~np.isnan(result["radii"])))
        vis = draw_polar(frame, result, cal)
        cv2.putText(vis, "%d/%d directions  thr %.1f  %.1f fps"
                    % (n_hit, N_RAYS, scanner.threshold, fps),
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow(win, vis)
        cv2.imshow("panorama", draw_pano(result, scanner))

        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("c"):
            stamp = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite("scan_%s_raw.png" % stamp, vis)
            cv2.imwrite("scan_%s_pano.png" % stamp, draw_pano(result, scanner))
            with open("scan_%s.json" % stamp, "w") as f:
                json.dump(dict(calib=cal.to_dict(), scan=to_records(result, cal)),
                          f, indent=2)
            print("captured scan_%s.*" % stamp)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
