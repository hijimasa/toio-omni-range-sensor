#!/usr/bin/env python3
"""Shared pieces for the omnidirectional range sensor tools.

Frame sources, the annulus -> panorama unwrapping, the calibration file and the
ignore mask all live here so the viewer, the mask editor and the range scanner
agree on exactly one definition of each.

Conventions
-----------
Calibration pixel coordinates are whatever resolution the calibration was taken
at (VGA by default); `Calib.scaled_to()` rescales them for a larger frame.

With the calibration this project uses (`theta0=90, ccw=true`) a panorama
*column angle* is the ROS bearing directly: x forward, CCW positive.

Panorama row 0 is the inner edge of the annulus (near the robot) and the last
row is the outer edge (far), unless `flip_radial` is set.
"""

import json
import os
import threading
import time
import urllib.request

import cv2
import numpy as np

CALIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calib")
DEFAULT_CALIB = os.path.join(CALIB_DIR, "mirror_calib.json")
DEFAULT_MASK = os.path.join(CALIB_DIR, "ignore_mask.png")

N_RAYS = 72
RAY_STEP_DEG = 360.0 / N_RAYS
PANO_W = 720          # 0.5 deg per column; 10 columns per measurement direction
PANO_H = 200


# ----------------------------------------------------------------- sources ---

class MjpegStream:
    """Reads a multipart MJPEG stream in a thread, keeping only the newest frame.

    cv2.VideoCapture can open these URLs but buffers several frames, which makes
    the overlay lag behind the mirror while you are aiming it.
    """

    def __init__(self, url, timeout=5.0, stall_timeout=4.0):
        self.url = url
        self.timeout = timeout
        # A congested 2.4 GHz link often goes quiet without closing the socket,
        # so a silent stream is treated as dead and reconnected.
        self.stall_timeout = stall_timeout
        self.reconnects = 0
        self._frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._resp = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            resp = None
            try:
                resp = urllib.request.urlopen(self.url, timeout=self.timeout)
                self._resp = resp
                buf = b""
                t_last = time.time()
                while not self._stop.is_set():
                    chunk = resp.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    if time.time() - t_last > self.stall_timeout:
                        raise IOError("stream stalled for %.0fs" % self.stall_timeout)
                    # Scan for a complete JPEG rather than parsing the multipart
                    # headers; SOI/EOI markers are enough and tolerate garbage.
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9", start + 2)
                    if start != -1 and end != -1:
                        jpg = buf[start:end + 2]
                        buf = buf[end + 2:]
                        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            t_last = time.time()
                            with self._lock:
                                self._frame = img
                    if len(buf) > 1 << 20:      # desynchronised, drop it
                        buf = b""
            except Exception as exc:
                if self._stop.is_set():
                    break        # release() closed the socket under us; not an error
                self.reconnects += 1
                print("stream error: %s (reconnect #%d)" % (exc, self.reconnects))
                time.sleep(1.0)
            finally:
                if resp is not None:
                    try:
                        resp.close()
                    except Exception:
                        pass

    def read(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def release(self):
        # The ESP32 keeps the stream socket allocated until the client closes
        # it, and a leaked one blocks the next connection, so close explicitly.
        self._stop.set()
        try:
            if self._resp is not None:
                self._resp.close()
        except Exception:
            pass
        self._thread.join(timeout=2.0)


class StillPoller:
    """Polls /still instead of holding one long MJPEG connection.

    Every frame is an independent request, so a dropped packet costs one frame
    rather than the whole session. Slower, but it is the option that survives a
    congested 2.4 GHz link.
    """

    def __init__(self, url, rate=4.0, timeout=4.0):
        self.url = url
        self.period = 1.0 / max(rate, 0.1)
        self.timeout = timeout
        self.reconnects = 0
        self._frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            t0 = time.time()
            try:
                with urllib.request.urlopen(self.url, timeout=self.timeout) as r:
                    data = r.read()
                img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    with self._lock:
                        self._frame = img
            except Exception as exc:
                self.reconnects += 1
                print("still error: %s (retry #%d)" % (exc, self.reconnects))
            time.sleep(max(0.0, self.period - (time.time() - t0)))

    def read(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def release(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


class FileSource:
    """A still image or a video file / device, behind the same read() API."""

    def __init__(self, path):
        self.still = None
        self.cap = None
        self.reconnects = 0
        img = cv2.imread(path) if not str(path).isdigit() else None
        if img is not None:
            self.still = img
        else:
            self.cap = cv2.VideoCapture(int(path) if str(path).isdigit() else path)
            if not self.cap.isOpened():
                raise SystemExit("cannot open source: %s" % path)

    def read(self):
        if self.still is not None:
            return self.still.copy()
        ok, frame = self.cap.read()
        if not ok:                      # loop video files
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        return frame if ok else None

    def release(self):
        if self.cap is not None:
            self.cap.release()


def add_source_args(ap, default_url=None):
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--url", help="MJPEG stream URL, e.g. http://<ip>:81/stream")
    src.add_argument("--still-url", help="poll this /still URL instead of streaming; "
                                         "slower but far more robust on a flaky link")
    src.add_argument("--source", help="image file, video file, or webcam index")
    ap.add_argument("--still-rate", type=float, default=4.0,
                    help="polls per second for --still-url (default 4)")
    ap.set_defaults(_default_url=default_url)


def open_source(args):
    if args.url:
        return MjpegStream(args.url)
    if args.still_url:
        return StillPoller(args.still_url, args.still_rate)
    if args.source:
        return FileSource(args.source)
    if getattr(args, "_default_url", None):
        return MjpegStream(args._default_url)
    raise SystemExit("give one of --url / --still-url / --source")


def first_frame(cap, timeout=20.0):
    print("waiting for the first frame ...")
    t0 = time.time()
    while True:
        frame = cap.read()
        if frame is not None:
            return frame
        if time.time() - t0 > timeout:
            # Release before bailing out. The ESP32 holds the stream socket
            # until the client closes it, so giving up without closing leaks
            # one on every failed run and makes the next attempt worse.
            try:
                cap.release()
            except Exception:
                pass
            raise SystemExit("no frame received. The unit answers /status and "
                             "ping but its bulk throughput collapses after a "
                             "few hours up -- power-cycle it and retry.")
        time.sleep(0.05)


# --------------------------------------------------------- camera framesize ---

# The esp32-camera framesize enum, as of the version arduino-esp32 3.3.7 ships.
# Two things make this worth spelling out rather than passing a bare number:
# the values move between library versions (VGA is 10 here, 8 on older ones),
# and several modes are a crop of the sensor rather than a scaled view of the
# same field. A calibration does not carry across either change, so a tool that
# selects a size has to check what actually came back.
#   name, width, height, is_crop
FRAMESIZES = {
    5:  ("240X240",  240,  240, True),
    6:  ("QVGA",     320,  240, False),
    7:  ("320X320",  320,  320, True),
    8:  ("CIF",      400,  296, False),
    9:  ("HVGA",     480,  320, True),
    10: ("VGA",      640,  480, False),
    11: ("SVGA",     800,  600, False),
    12: ("XGA",     1024,  768, False),
    13: ("HD",      1280,  720, True),
    14: ("SXGA",    1280, 1024, True),
    15: ("UXGA",    1600, 1200, False),
}


def add_framesize_arg(ap):
    ap.add_argument("--set-framesize", type=int, default=None, metavar="N",
                    help="switch the unit to this framesize first: "
                         + ", ".join("%d=%s %dx%d%s"
                                     % (k, v[0], v[1], v[2], " CROP" if v[3] else "")
                                     for k, v in sorted(FRAMESIZES.items())
                                     if k in (6, 7, 8, 10, 12))
                         + ". Sizes marked CROP are a sensor crop, not a scaled "
                           "view -- a calibration never carries between them.")


def source_host(args):
    """Host to send /ctrl to. The stream lives on :81, the control server on 80."""
    base = (getattr(args, "url", None) or getattr(args, "still_url", None)
            or getattr(args, "_default_url", None) or "")
    if "//" not in base:
        return None
    return base.split("/")[2].split(":")[0]


def set_framesize(args, n, settle=2.0):
    """Pin the unit's framesize, before the source is opened.

    Done from the tool rather than by hand because the calibration is only
    valid at the size it was taken on: pinning it with the same command that
    starts the measurement is the only way the two cannot drift apart.
    """
    if n is None:
        return None
    host = source_host(args)
    if host is None:
        print("note: --set-framesize needs --url or --still-url; ignoring it")
        return None
    urllib.request.urlopen("http://%s/ctrl?var=framesize&val=%d" % (host, n),
                           timeout=5).read()
    print("framesize -> %d (%s)" % (n, FRAMESIZES.get(n, ("?",))[0]))
    # The sensor needs a moment, and /still keeps handing back two or three
    # frames from the old buffer immediately after the change.
    time.sleep(settle)
    try:
        st = json.loads(urllib.request.urlopen("http://%s/status" % host,
                                               timeout=5).read().decode())
        got = int(st.get("framesize", -1))
        if got != n:
            print("WARNING: the unit reports framesize %d, not %d" % (got, n))
        return got
    except Exception as e:                     # noqa: BLE001 - diagnostic only
        print("note: could not read /status back (%s)" % e)
        return None


def first_frame_at(cap, n, timeout=20.0):
    """First frame that is really the size framesize `n` should produce.

    The unit applies the change at once -- /status agrees immediately -- but the
    stream keeps serving buffers captured at the old size for a frame or two
    (measured on this unit: after switching to CIF the next frame delivered was
    still 640x480). Taking whatever arrives first would calibrate the new
    geometry against the old picture, and nothing about the result would look
    wrong. So drop frames until the size matches, and say so if it never does --
    the enum value itself moves between esp32-camera versions.
    """
    frame = first_frame(cap, timeout)
    if n is None or n not in FRAMESIZES:
        return frame
    name, w, h, _ = FRAMESIZES[n]
    # read() hands back the newest frame every call, not the next one, so this
    # polls on a timer rather than counting reads -- a spin here would "drop"
    # tens of thousands of copies of the same frame.
    t0 = time.time()
    while (frame.shape[1], frame.shape[0]) != (w, h):
        if time.time() - t0 > timeout:
            print("WARNING: asked for framesize %d (%s, %dx%d) but the frames "
                  "are still %dx%d after %.0f s. Check the framesize enum in "
                  "sensor.h for this build before calibrating anything."
                  % (n, name, w, h, frame.shape[1], frame.shape[0], timeout))
            return frame
        time.sleep(0.05)
        nxt = cap.read()
        if nxt is not None:
            frame = nxt
    waited = time.time() - t0
    if waited > 0.15:
        print("waited %.1f s for the stream to catch up with the size change"
              % waited)
    return frame


# ------------------------------------------------------------- calibration ---

class Calib:
    def __init__(self, cx, cy, r_in, r_out, theta0=0.0, ccw=False, flip_radial=False,
                 frame_width=None, frame_height=None):
        # The framesize the calibration was taken at. Some camera modes are a
        # crop of the sensor rather than a scaled version of the same view, so
        # a calibration is only valid for the size it was made on -- applying
        # it to another size silently produces plausible but wrong radii.
        self.frame_width, self.frame_height = frame_width, frame_height
        self.cx, self.cy = float(cx), float(cy)
        self.r_in, self.r_out = float(r_in), float(r_out)
        self.theta0 = float(theta0)
        self.ccw = bool(ccw)              # direction of increasing angle index
        self.flip_radial = bool(flip_radial)   # put r_out on the top panorama row

    def to_dict(self):
        d = dict(cx=self.cx, cy=self.cy, r_in=self.r_in, r_out=self.r_out,
                 theta0=self.theta0, ccw=self.ccw, flip_radial=self.flip_radial,
                 n_rays=N_RAYS)
        if self.frame_width:
            d["frame_width"] = int(self.frame_width)
            d["frame_height"] = int(self.frame_height or self.frame_width)
        return d

    @staticmethod
    def from_dict(d):
        return Calib(d["cx"], d["cy"], d["r_in"], d["r_out"],
                     d.get("theta0", 0.0), d.get("ccw", False),
                     d.get("flip_radial", False),
                     d.get("frame_width"), d.get("frame_height"))

    def scaled_to(self, frame_width, calib_width=640.0):
        """Same calibration expressed in the pixels of a larger/smaller frame."""
        s = frame_width / float(calib_width)
        if abs(s - 1.0) < 1e-6:
            return self
        return Calib(self.cx * s, self.cy * s, self.r_in * s, self.r_out * s,
                     self.theta0, self.ccw, self.flip_radial,
                     self.frame_width, self.frame_height)

    def angles_deg(self):
        """Bearing of each of the 72 rays, in image coordinates."""
        sign = -1.0 if self.ccw else 1.0
        return self.theta0 + sign * np.arange(N_RAYS) * RAY_STEP_DEG

    def radius_of_row(self, row, height):
        """Source-frame radius (px) a panorama row was sampled from."""
        f = row / float(max(height - 1, 1))
        if self.flip_radial:
            f = 1.0 - f
        return self.r_in + (self.r_out - self.r_in) * f


def check_frame(cal, frame, fatal=True):
    """Refuse frames that the calibration was not made for."""
    if not cal.frame_width:
        return
    h, w = frame.shape[:2]
    if (w, h) == (int(cal.frame_width), int(cal.frame_height or cal.frame_width)):
        return
    msg = ("frame is %dx%d but the calibration was taken at %dx%d. Some camera "
           "modes crop rather than scale, so the calibration does not carry "
           "over -- switch the camera back (/ctrl?var=framesize) or re-calibrate."
           % (w, h, cal.frame_width, cal.frame_height or cal.frame_width))
    if fatal:
        raise SystemExit("ERROR: " + msg)
    print("WARNING: " + msg)


def load_calib(path=DEFAULT_CALIB):
    with open(path) as f:
        return Calib.from_dict(json.load(f))


def save_calib(cal, path=DEFAULT_CALIB):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        json.dump(cal.to_dict(), f, indent=2)


# ------------------------------------------------------------- unwrapping ---

def build_maps(cal, width=PANO_W, height=PANO_H):
    """remap tables for annulus -> (angle, radius) panorama."""
    sign = -1.0 if cal.ccw else 1.0
    ang = np.deg2rad(cal.theta0 + sign * np.linspace(0.0, 360.0, width, endpoint=False))
    rad = np.linspace(cal.r_in, cal.r_out, height)
    if cal.flip_radial:
        rad = rad[::-1]
    a, r = np.meshgrid(ang, rad)
    map_x = (cal.cx + r * np.cos(a)).astype(np.float32)
    map_y = (cal.cy + r * np.sin(a)).astype(np.float32)
    return map_x, map_y


def unwrap(frame, map_x, map_y):
    return cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))


def coverage(cal, shape):
    """Fraction of the 360 deg whose outer sample point falls inside the frame."""
    h, w = shape[:2]
    a = np.deg2rad(np.arange(0.0, 360.0, 0.5))
    x = cal.cx + cal.r_out * np.cos(a)
    y = cal.cy + cal.r_out * np.sin(a)
    inside = (x >= 0) & (x < w) & (y >= 0) & (y < h)
    return float(inside.mean())


def col_to_bearing(col, width=PANO_W):
    """Panorama column -> bearing in degrees, wrapped to (-180, 180]."""
    return (col * 360.0 / width + 180.0) % 360.0 - 180.0


def bearing_to_col(deg, width=PANO_W):
    return int(round((deg % 360.0) * width / 360.0)) % width


# ------------------------------------------------------------- ignore mask ---

def load_mask(path=DEFAULT_MASK, width=PANO_W, height=PANO_H):
    """Panorama-space mask. True = usable, False = ignore.

    Preferred over a per-direction inner radius: the camera's own reflection,
    the mount and any cable are not a single radius per direction, and a painted
    region can describe all of them (and be corrected by hand).
    """
    if not os.path.exists(path):
        return np.ones((height, width), bool)
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return np.ones((height, width), bool)
    if img.shape != (height, width):
        img = cv2.resize(img, (width, height), interpolation=cv2.INTER_NEAREST)
    return img > 127


def save_mask(mask, path=DEFAULT_MASK):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    cv2.imwrite(path, (mask.astype(np.uint8) * 255))


def auto_mask(pano, margin_rows=6):
    """First guess at the ignore region: the dark blob the camera makes of itself.

    Walks outward per column until the brightness reaches a fraction of this
    image's own floor level. The threshold has to be relative -- with a fixed
    one, a darker exposure alone looks like a bigger dead zone (that mistake
    cost a wrong "removing the cable made it worse" reading once).
    """
    v = cv2.cvtColor(pano, cv2.COLOR_BGR2HSV)[:, :, 2].astype(np.float32)
    v = cv2.GaussianBlur(v, (1, 21), 0)
    h, w = v.shape
    floor = float(np.median(v[int(h * 0.6):, :]))
    th = 0.72 * floor

    mask = np.ones((h, w), bool)
    for c in range(w):
        idx = np.where(v[:, c] > th)[0]
        edge = idx[0] if len(idx) else h
        mask[:min(edge + margin_rows, h), c] = False
    return mask


def holder_mask(pano, hue_range=(15, 40), sat_min=70, val_min=70,
                inner_frac=0.4, arm_sat_rise=15.0, grow=6):
    """Ignore region for a coloured mount: the base around the lens and the
    arms that hold the mirror.

    auto_mask() only sees brightness, and a yellow holder is as bright as the
    floor -- it comes through as "usable" and then as an obstacle in every
    scan. So this keys on hue instead. Two kinds of feature:

    - the base around the lens, reflected at the middle of the mirror: per
      column, everything from r_in out to the last coloured row inside the
      inner part of the annulus is closed.
    - the arms, which stand between camera and mirror and block the whole
      radius: a column whose saturation stands clearly above the floor across
      the middle of the annulus is closed top to bottom. The test is relative
      (floor + arm_sat_rise) because the arm facing away from the light is
      only weakly coloured -- an absolute hue test misses it.

    Returns True = usable, like auto_mask().
    """
    hsv = cv2.cvtColor(pano, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1].astype(np.float32), hsv[:, :, 2]
    lo, hi = hue_range
    hue_ok = (h >= lo) & (h <= hi) if lo <= hi else (h >= lo) | (h <= hi)
    col = hue_ok & (s >= sat_min) & (v >= val_min)
    if grow:
        k = np.ones((2 * grow + 1, 2 * grow + 1), np.uint8)
        col = cv2.dilate(col.astype(np.uint8), k) > 0
    hh, w = col.shape
    mask = np.ones((hh, w), bool)

    inner = int(hh * inner_frac)
    for c in range(w):
        idx = np.where(col[:inner, c])[0]
        if len(idx):
            mask[:idx[-1] + 1, c] = False

    mid = s[int(hh * 0.2):int(hh * 0.8), :]
    prof = np.median(mid, axis=0)
    # smooth over +-0.5 deg; the floor level is the median column
    prof = np.convolve(np.r_[prof[-1:], prof, prof[:1]], np.ones(3) / 3.0, "same")[1:-1]
    arm = prof > np.median(prof) + arm_sat_rise
    if grow:
        arm = np.convolve(np.r_[arm[-grow:], arm, arm[:grow]].astype(int),
                          np.ones(2 * grow + 1), "same")[grow:-grow] > 0
    mask[:, arm] = False
    return mask


def far_mask(cal, r_far, height=PANO_H, width=PANO_W):
    """Close every row sampled past r_far (calibration px). On a hemisphere the
    outer band looks at the horizon, where a pixel is tens of cm; the far end
    of the scan is set here, not by r_out."""
    rows = np.array([cal.radius_of_row(r, height) for r in range(height)])
    mask = np.ones((height, width), bool)
    mask[rows > r_far, :] = False
    return mask


# ------------------------------------------------------------- range fit ---

DEFAULT_RANGE_FIT = os.path.join(CALIB_DIR, "range_fit.json")


class RangeFit:
    """radius (calibration px) -> distance (cm), from fit_range.py.

    r = a + b*arctan(d/c), so d = c*tan((r-a)/b). The arctan saturates at the
    horizon, which is what makes radii past the mirror rim come back as NaN
    instead of some large but finite number.
    """

    def __init__(self, a, b, c, d_min=None, d_max=None,
                 calib_r_in=None, calib_r_out=None):
        self.a, self.b, self.c = float(a), float(b), float(c)
        self.d_min, self.d_max = d_min, d_max
        self.calib_r_in, self.calib_r_out = calib_r_in, calib_r_out

    def matches(self, cal, tol=1.0):
        """Was this fit taken on the calibration now in use?

        Distances from a fit made on a different annulus are silently wrong,
        which is worse than having no distances at all.
        """
        if self.calib_r_in is None or self.calib_r_out is None:
            return None                      # older fit file, cannot tell
        return (abs(self.calib_r_in - cal.r_in) <= tol
                and abs(self.calib_r_out - cal.r_out) <= tol)

    @staticmethod
    def load(path=DEFAULT_RANGE_FIT):
        if not os.path.exists(path):
            return None
        with open(path) as f:
            d = json.load(f)
        return RangeFit(d["a"], d["b"], d["c"], d.get("d_min"), d.get("d_max"),
                        d.get("calib_r_in"), d.get("calib_r_out"))

    def distance_cm(self, radius_px):
        t = (np.asarray(radius_px, float) - self.a) / self.b
        out = np.full(np.shape(t), np.nan, float)
        ok = np.isfinite(t) & (t > 0) & (t < np.pi / 2 * 0.999)
        out[ok] = self.c * np.tan(t[ok])
        return out

    def extrapolated(self, distance_cm):
        """True where the answer is outside the range the fit was measured on."""
        d = np.asarray(distance_cm, float)
        lo = self.d_min if self.d_min is not None else -np.inf
        hi = self.d_max if self.d_max is not None else np.inf
        return np.isfinite(d) & ((d < lo) | (d > hi))


# ---------------------------------------------------------------- drawing ---

def draw_overlay(frame, cal):
    vis = frame.copy()
    c = (int(round(cal.cx)), int(round(cal.cy)))
    lw = max(1, frame.shape[1] // 500)

    for deg in cal.angles_deg():
        a = np.deg2rad(deg)
        p0 = (int(cal.cx + cal.r_in * np.cos(a)), int(cal.cy + cal.r_in * np.sin(a)))
        p1 = (int(cal.cx + cal.r_out * np.cos(a)), int(cal.cy + cal.r_out * np.sin(a)))
        major = abs(deg - cal.theta0) % 30.0 < 1e-6
        cv2.line(vis, p0, p1, (0, 255, 255) if major else (110, 110, 60),
                 lw if major else 1, cv2.LINE_AA)

    cv2.circle(vis, c, int(cal.r_out), (0, 255, 0), lw, cv2.LINE_AA)
    cv2.circle(vis, c, int(cal.r_in), (0, 165, 255), lw, cv2.LINE_AA)
    cv2.drawMarker(vis, c, (0, 0, 255), cv2.MARKER_CROSS, 14, lw)

    for k in range(0, 360, 30):
        a = np.deg2rad(cal.theta0 + (-k if cal.ccw else k))
        p = (int(cal.cx + (cal.r_out + 14) * np.cos(a)),
             int(cal.cy + (cal.r_out + 14) * np.sin(a)))
        cv2.putText(vis, str(k), p, cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 255, 255), 1, cv2.LINE_AA)
    return vis


def annotate_panorama(pano, label_step=30):
    h, w = pano.shape[:2]
    for i in range(N_RAYS):
        x = int(round(i * w / N_RAYS))
        major = (i % (label_step // 5) == 0)
        cv2.line(pano, (x, 0), (x, h), (0, 255, 255) if major else (90, 90, 50), 1)
        if major:
            cv2.putText(pano, "%d" % int(col_to_bearing(x, w)), (x + 2, 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)
    return pano


def auto_circle(frame):
    """Rough guess at the mirror rim: the strongest Hough circle in the frame."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    h, w = gray.shape
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=2,
                               minDist=min(h, w) // 2,
                               param1=120, param2=60,
                               minRadius=int(min(h, w) * 0.15),
                               maxRadius=int(min(h, w) * 0.55))
    if circles is None:
        return None
    x, y, r = circles[0][0]
    return float(x), float(y), float(r)
