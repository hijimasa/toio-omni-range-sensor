#!/usr/bin/env python3
"""Check that the convex mirror on the Timer Camera F really gives a 360 deg view.

Shows, side by side:

  raw       the live frame with the mirror annulus and 72 rays (5 deg) drawn on it
  panorama  the annulus unwrapped to angle x radius
  sat       the saturation channel of the panorama, which is the cue the range
            sensor thresholds on

Adjust cx / cy / r_inner / r_outer / theta0 with the trackbars until the green
circle sits on the mirror rim and the orange one on the camera's own reflection.
The header line reports whether the full annulus is inside the frame -- that is
the actual pass/fail for "is the whole 360 deg captured".

The direction sign (`ccw`) cannot be settled from a still image: fit it by
moving a known object through a known angle and checking which way the panorama
moves. Getting this wrong still produces a plausible-looking fit.

  ./omni_viewer.py --url http://<ip>:81/stream
  ./omni_viewer.py --source shot.jpg
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from omni_common import (DEFAULT_CALIB, PANO_H, PANO_W, Calib, add_source_args,
                         annotate_panorama, auto_circle, build_maps, coverage,
                         draw_overlay, first_frame, load_calib, open_source,
                         save_calib, unwrap, check_frame)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_source_args(ap, default_url="http://192.168.4.1:81/stream")
    ap.add_argument("--calib", default=DEFAULT_CALIB, help="calibration JSON path")
    ap.add_argument("--pano-width", type=int, default=PANO_W,
                    help="panorama width in px (0.5 deg/px at 720)")
    ap.add_argument("--pano-height", type=int, default=PANO_H)
    ap.add_argument("--outdir", default="captures", help="where 'c' writes snapshots")
    args = ap.parse_args()

    cap = open_source(args)
    frame = first_frame(cap)
    h, w = frame.shape[:2]
    print("frame %dx%d" % (w, h))

    if os.path.exists(args.calib):
        loaded = load_calib(args.calib)
        check_frame(loaded, frame, fatal=False)
        cal = loaded.scaled_to(w)
        print("loaded %s" % args.calib)
    else:
        cal = Calib(w / 2, h / 2, min(w, h) * 0.08, min(w, h) * 0.45)

    win = "raw (s=save c=capture a=auto q=quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.namedWindow("panorama", cv2.WINDOW_NORMAL)
    cv2.namedWindow("saturation", cv2.WINDOW_NORMAL)
    rmax = int(np.hypot(w, h) / 2)
    nop = lambda v: None
    cv2.createTrackbar("cx", win, int(cal.cx), w, nop)
    cv2.createTrackbar("cy", win, int(cal.cy), h, nop)
    cv2.createTrackbar("r_in", win, int(cal.r_in), rmax, nop)
    cv2.createTrackbar("r_out", win, int(cal.r_out), rmax, nop)
    cv2.createTrackbar("theta0", win, int(cal.theta0) % 360, 359, nop)
    cv2.createTrackbar("ccw", win, int(cal.ccw), 1, nop)
    cv2.createTrackbar("flip_r", win, int(cal.flip_radial), 1, nop)

    maps = None
    last_key = None
    t_prev, fps = time.time(), 0.0

    while True:
        new = cap.read()
        if new is not None:
            frame = new
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6)
            t_prev = now

        cal.cx = cv2.getTrackbarPos("cx", win)
        cal.cy = cv2.getTrackbarPos("cy", win)
        cal.r_in = cv2.getTrackbarPos("r_in", win)
        cal.r_out = max(cv2.getTrackbarPos("r_out", win), cal.r_in + 1)
        cal.theta0 = cv2.getTrackbarPos("theta0", win)
        cal.ccw = bool(cv2.getTrackbarPos("ccw", win))
        cal.flip_radial = bool(cv2.getTrackbarPos("flip_r", win))

        key = (cal.cx, cal.cy, cal.r_in, cal.r_out, cal.theta0, cal.ccw, cal.flip_radial)
        if key != last_key or maps is None:
            maps = build_maps(cal, args.pano_width, args.pano_height)
            last_key = key

        vis = draw_overlay(frame, cal)
        cov = coverage(cal, frame.shape)
        ok = cov > 0.999
        drops = getattr(cap, "reconnects", 0)

        # Centring aid. r_out can never exceed the distance from the mirror
        # centre to the nearest frame edge, so an off-centre mirror throws away
        # radial resolution on the far side. Magenta marks where the mirror
        # should sit for the annulus to be as wide as the optics allow.
        fh, fw = frame.shape[:2]
        fcx, fcy = fw / 2.0, fh / 2.0
        r_max_now = min(cal.cx, cal.cy, fw - cal.cx, fh - cal.cy)
        r_max_centred = min(fcx, fcy)
        cv2.drawMarker(vis, (int(fcx), int(fcy)), (255, 0, 255),
                       cv2.MARKER_TILTED_CROSS, 18, 2)
        cv2.circle(vis, (int(fcx), int(fcy)), int(r_max_centred), (255, 0, 255), 1, cv2.LINE_AA)
        cv2.arrowedLine(vis, (int(cal.cx), int(cal.cy)), (int(fcx), int(fcy)),
                        (255, 0, 255), 2, cv2.LINE_AA, tipLength=0.2)

        cv2.putText(vis, "360deg inside frame: %s (%.1f%%)  %.1f fps  drops %d"
                    % ("YES" if ok else "NO", cov * 100.0, fps, drops),
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if ok else (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(vis, "offset from frame centre: %+.0f,%+.0f px   r_out max now %.0f"
                    " / centred %.0f   annulus %.0f -> %.0f"
                    % (cal.cx - fcx, cal.cy - fcy, r_max_now, r_max_centred,
                       cal.r_out - cal.r_in, r_max_centred - cal.r_in),
                    (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA)

        pano = unwrap(frame, *maps)
        sat = cv2.cvtColor(pano, cv2.COLOR_BGR2HSV)[:, :, 1]
        sat_vis = cv2.applyColorMap(sat, cv2.COLORMAP_TURBO)

        cv2.imshow(win, vis)
        cv2.imshow("panorama", annotate_panorama(pano.copy()))
        cv2.imshow("saturation", annotate_panorama(sat_vis))

        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("s"):
            # Store in the calibration's own pixel scale, not this frame's.
            out = cal.scaled_to(640.0, calib_width=frame.shape[1])
            out.frame_width, out.frame_height = frame.shape[1], frame.shape[0]
            save_calib(out, args.calib)
            print("saved %s (valid for %dx%d frames)"
                  % (args.calib, out.frame_width, out.frame_height))
        elif k == ord("c"):
            os.makedirs(args.outdir, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            for name, im in (("raw", frame), ("overlay", vis),
                             ("pano", pano), ("sat", sat_vis)):
                cv2.imwrite(os.path.join(args.outdir, "%s_%s.png" % (stamp, name)), im)
            print("captured %s/%s_*.png" % (args.outdir, stamp))
        elif k == ord("a"):
            found = auto_circle(frame)
            if found is None:
                print("no circle found")
            else:
                x, y, r = found
                print("auto: cx=%.0f cy=%.0f r=%.0f" % (x, y, r))
                cv2.setTrackbarPos("cx", win, int(x))
                cv2.setTrackbarPos("cy", win, int(y))
                cv2.setTrackbarPos("r_out", win, int(r))
                cv2.setTrackbarPos("r_in", win, int(r * 0.18))

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
