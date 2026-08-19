#!/usr/bin/env python3
"""Paint the region of the panorama the range scan must ignore.

The camera's own reflection, the mount and any cable are fixed features of the
image, not obstacles. They are not one inner radius per direction either -- a
cable crosses several directions at different radii -- so they are described as
a painted mask in panorama space (angle x radius) instead.

Start from a view with the floor clear of obstacles: press `a` and the dark blob
the camera makes of itself is filled in automatically, then brush in whatever it
missed.

  ./mask_editor.py --url http://<ip>:81/stream

  left drag    mark as ignored
  right drag   mark as usable again
  a            auto-fill from the current frame
  [ / ]        brush smaller / larger
  r            reset to all-usable
  s            save to calib/ignore_mask.png
  q            quit
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from omni_common import (DEFAULT_CALIB, DEFAULT_MASK, PANO_H, PANO_W,
                         add_source_args, annotate_panorama, auto_mask,
                         build_maps, first_frame, load_calib, load_mask,
                         open_source, save_mask, unwrap, check_frame,
                         add_framesize_arg, set_framesize, first_frame_at)

ZOOM = 2          # the panorama is only 200 px tall; painting it 1:1 is fiddly


class Painter:
    def __init__(self, mask):
        self.mask = mask
        self.brush = 12
        self.drawing = 0      # 1 = paint ignore, 2 = paint usable

    def on_mouse(self, event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = 1
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.drawing = 2
        elif event in (cv2.EVENT_LBUTTONUP, cv2.EVENT_RBUTTONUP):
            self.drawing = 0
        if self.drawing and event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONDOWN,
                                      cv2.EVENT_RBUTTONDOWN):
            cv2.circle(self.mask_u8, (x // ZOOM, y // ZOOM), self.brush,
                       0 if self.drawing == 1 else 255, -1)

    @property
    def mask_u8(self):
        return self._u8

    @mask_u8.setter
    def mask_u8(self, v):
        self._u8 = v


def render(pano, mask_u8, brush):
    vis = pano.copy()
    ignored = mask_u8 < 128
    # magenta wash over everything the scan will skip
    vis[ignored] = (vis[ignored] * 0.45 + np.array([120, 0, 120]) * 0.55).astype(np.uint8)
    vis = annotate_panorama(vis)
    vis = cv2.resize(vis, (vis.shape[1] * ZOOM, vis.shape[0] * ZOOM),
                     interpolation=cv2.INTER_NEAREST)
    pct = 100.0 * ignored.mean()
    cv2.putText(vis, "ignored %.1f%%   brush %d   a=auto s=save r=reset q=quit"
                % (pct, brush), (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    return vis


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_source_args(ap)
    ap.add_argument("--calib", default=DEFAULT_CALIB)
    ap.add_argument("--mask", default=DEFAULT_MASK)
    add_framesize_arg(ap)
    args = ap.parse_args()

    # The mask is in panorama coordinates, which come from the calibration,
    # which is only valid at one frame size -- so this has to be pinned too.
    set_framesize(args, args.set_framesize)

    cal = load_calib(args.calib)
    cap = open_source(args)
    frame = first_frame_at(cap, args.set_framesize)
    check_frame(cal, frame)
    maps = build_maps(cal.scaled_to(frame.shape[1]), PANO_W, PANO_H)

    mask = load_mask(args.mask, PANO_W, PANO_H)
    p = Painter(mask)
    p.mask_u8 = (mask.astype(np.uint8) * 255)

    win = "ignore mask"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, p.on_mouse)

    frame_w = frame.shape[1]
    while True:
        new = cap.read()
        if new is not None:
            frame = new
            if frame.shape[1] != frame_w:      # resolution changed under us
                frame_w = frame.shape[1]
                maps = build_maps(cal.scaled_to(frame_w), PANO_W, PANO_H)
        pano = unwrap(frame, *maps)
        cv2.imshow(win, render(pano, p.mask_u8, p.brush))

        k = cv2.waitKey(20) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("a"):
            auto = auto_mask(pano)
            # union: keep whatever was already painted by hand
            p.mask_u8 = np.minimum(p.mask_u8, auto.astype(np.uint8) * 255)
            print("auto-filled: %.1f%% ignored" % (100.0 * (p.mask_u8 < 128).mean()))
        elif k == ord("r"):
            p.mask_u8 = np.full((PANO_H, PANO_W), 255, np.uint8)
        elif k == ord("["):
            p.brush = max(1, p.brush - 2)
        elif k == ord("]"):
            p.brush = min(60, p.brush + 2)
        elif k == ord("s"):
            save_mask(p.mask_u8 > 127, args.mask)
            print("saved %s (%.1f%% ignored)"
                  % (args.mask, 100.0 * (p.mask_u8 < 128).mean()))

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
