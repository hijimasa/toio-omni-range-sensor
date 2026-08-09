#!/usr/bin/env python3
"""Render a fake catadioptric frame so omni_viewer.py can be exercised without
the hardware.

A ring of coloured wedges stands in for the surroundings; the dark disc in the
middle is the camera's own reflection off the mirror apex. Feed the result to
the viewer with --source and the unwrapped panorama should show the wedges as
straight vertical bands in the right order.

    ./make_synthetic_omni.py -o /tmp/omni.png
    ./omni_viewer.py --source /tmp/omni.png
"""

import argparse

import cv2
import numpy as np


def render(width, height, cx, cy, r_in, r_out, n_wedges=12):
    img = np.full((height, width, 3), 25, np.uint8)

    yy, xx = np.mgrid[0:height, 0:width].astype(np.float32)
    r = np.hypot(xx - cx, yy - cy)
    a = (np.degrees(np.arctan2(yy - cy, xx - cx)) + 360.0) % 360.0

    ring = (r >= r_in) & (r <= r_out)
    idx = (a / (360.0 / n_wedges)).astype(np.int32)

    # Hue sweeps once around the ring, value dims with radius to mimic the
    # ground getting darker towards the horizon.
    hsv = np.zeros((height, width, 3), np.uint8)
    hsv[..., 0] = (idx * (180 // n_wedges)).astype(np.uint8)
    hsv[..., 1] = 200
    v = np.clip(255.0 - 120.0 * (r - r_in) / max(r_out - r_in, 1.0), 40, 255)
    hsv[..., 2] = v.astype(np.uint8)
    wedges = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    img[ring] = wedges[ring]

    # One wedge is left grey so the 0 deg reference is unambiguous.
    img[ring & (idx == 0)] = (200, 200, 200)

    img[r < r_in] = (15, 15, 15)
    cv2.circle(img, (int(cx), int(cy)), int(r_out), (0, 0, 0), 3, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--out", default="synthetic_omni.png")
    ap.add_argument("--size", type=int, nargs=2, default=[640, 480], metavar=("W", "H"))
    ap.add_argument("--wedges", type=int, default=12)
    args = ap.parse_args()

    w, h = args.size
    img = render(w, h, w / 2.0, h / 2.0, min(w, h) * 0.10, min(w, h) * 0.46,
                 args.wedges)
    cv2.imwrite(args.out, img)
    print("wrote %s (%dx%d)" % (args.out, w, h))


if __name__ == "__main__":
    main()
