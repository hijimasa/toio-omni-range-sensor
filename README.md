[English](README.md) | [日本語](README.ja.md)

# toio-omni-range-sensor

![Demo video](./figs/toio-omni-range.gif)

*Driving around obstacles under nav2. Left: RViz (LaserScan and path), right: the real robot. Current configuration (yellow holder, hemispherical mirror), real time, 8 fps.*

An omnidirectional range sensor for toio. A catadioptric system combining an M5Stack **Timer Camera F** with a convex mirror
captures the full 360 degrees in a single image, and the **saturation difference** between the floor and objects yields distances in 72 bearings at 5-degree steps.

Current status: **working on real hardware through Step 6** (the video above is the LaserScan shown in RViz)

| Step | Content | Status |
|---|---|---|
| 1 | Grab video from the Timer Camera F and confirm the convex mirror captures the full 360 degrees | Done |
| 2 | Calibration of the annular image -> panorama unwrapping (center, inner/outer radius) | Done (`omni_viewer.py`) |
| 3 | Saturation-based floor/object boundary detection -> radius in 72 bearings | Done (`omni_range.py`) |
| 4 | Radius -> distance conversion (calibration of the mirror shape) | Done (`fit_range.py`, 6 points, rms 0.9 px)|
| 5 | On-device processing and BLE output | Done (4.1 Hz, 0 dropped packets)|
| 6 | Output to ROS 2 (LaserScan) | Done (verified in RViz) |

## Current configuration

| Item | Value |
|---|---|
| Mirror | uxcell stainless-steel hollow hemisphere, about 60 mm in diameter |
| Holder | `stl/mirror_holder.stl` (yellow, 4 arms). Fixes the position of the mirror relative to the camera |
| Measurement framesize | **CIF 400x296** (RGB565). The fastest mode that still holds the full field of view |
| Calibration | `cx=265.6, cy=235.7, r_in=25.6, r_out=200` (in 640-width units) |
| Distance fit | `r(d) = -8.31 + 115.76*arctan(d/7.300)`, measured over 4.5 to 21 cm, rms 0.9 px |
| LaserScan range | **0.036 to 0.233 m** |
| Rate | 4.1 Hz (`scan_period` 200 ms) |
| Frame freshness | The frame being measured is about 98 ms old |

**The numbers in this table are tied to the measurement framesize**; the calibration does not carry over if you change the resolution (see below).

Where each number comes from is written in the steps below. **The mirror has been replaced three times: convex mirror -> cone -> stainless hemisphere.**
Measurements with the earlier mirrors and the designs that were not adopted (cone, hexagonal pyramid) are split out into [NOTES.md](NOTES.md).

---

## Layout

```
firmware/timercam_stream/   PlatformIO project for the Timer Camera F (MJPEG streaming)
tools/omni_common.py        Shared implementation: frame grabbing, panorama unwrapping, calibration, mask
tools/omni_viewer.py        Viewer. 360-degree check and mirror calibration (cx,cy,r_in,r_out,theta0)
tools/mask_editor.py        Creates the ignore mask (auto-generation plus hand correction)
tools/omni_range.py         Step 3 proper. Detects the floor/object boundary in 72 bearings
tools/fit_range.py           Step 4. Radius -> distance fit
tools/make_synthetic_omni.py  Synthetic image generator for trying the tools without hardware
tools/ble_client.py         CLI that receives scans over BLE
tools/gen_firmware_calib.py Calibration -> C header for the device and ROS parameters
calib/mirror_calib.json     Mirror calibration
calib/ignore_mask.png       Ignore mask (panorama coordinates, white = use / black = ignore)
stl/mirror_holder.stl       Mirror holder (for 3D printing)
ros2/toio_omni_range/       ROS 2 driver. BLE -> sensor_msgs/LaserScan
  toio_omni_range/omni_protocol.py  Packet definition (shared with ble_client.py)
ros2/toio_omni_bringup/     Bringup that starts together with toio_ros2, nav2 launch, collision_monitor, map of the mat
ros2/toio_omni_mapping/     Drive around the mat and build a map (occupancy grid with Position ID as ground truth)
docker/                     Container for ROS 2 (this repository itself does not need ROS)
```

## Using this repository in a ROS 2 workspace

**This repository can be placed directly under `colcon_ws/src/` (or added as a submodule).**
colcon only picks up directories that contain a `package.xml`, so `firmware/` `tools/` `calib/` are
ignored and only the 3 packages under `ros2/` get built.

```bash
cd colcon_ws/src
git submodule add https://github.com/<you>/toio-omni-range-sensor.git   # or git clone
pip install bleak            # the only dependency rosdep cannot resolve
cd .. && colcon build --symlink-install
```

| Package | Contents | Dependencies |
|---|---|---|
| `toio_omni_range` | Sensor driver. BLE -> `sensor_msgs/LaserScan` | rclpy, bleak |
| `toio_omni_bringup` | Bringup that starts the cube (`toio_ros2`) and the sensor together, nav2, collision_monitor | `toio_ros2`, navigation2 |
| `toio_omni_mapping` | Map building of the mat | `toio_omni_bringup`, nav2_map_server |

If you only want the sensor, `toio_omni_range` alone is enough, and the build passes without `toio_ros2`
(bringup and mapping declare it as exec_depend, so a missing dependency only matters at launch time).

`docker/` uses exactly this layout. If `toio_ros2_dev_kit` is checked out next to it,
the cube driver `toio_ros2` is mounted as well.

## Hardware

<img src="./figs/robot.jpg" width="320" alt="Timer Camera F on top of a toio, with the yellow holder and hemispherical mirror above it">

| Part | Role |
|---|---|
| toio Core Cube | Base vehicle. Everything sits on its flat top plate |
| M5Stack Timer Camera F (K034-F) | ESP32 + OV3660 + fisheye lens. Mounted with the lens pointing straight up |
| Mirror holder `stl/mirror_holder.stl` | 3D-printed part. Sits around the camera lens and supports the mirror with 4 arms |
| uxcell stainless-steel hollow hemisphere (about 60 mm in diameter, [Amazon](https://amzn.asia/d/084Gm1Y4)) | Placed on the holder's ring with the convex side facing down (toward the camera) |

Assembly, from the bottom up:

1. Stick the Timer Camera F to the toio top plate with **double-sided tape** (lens up).
2. Place the base of the holder on the camera and fix it with **hot glue**.
3. Place the mirror on the holder's ring and fix its rim with **hot glue**.

All holder and mirror joints are hot glue, so they can be removed and reattached. However,
**if the mirror-to-camera distance, tilt, or holder rotation changes, the calibration and the mask do not carry over.**
After reattaching, redo Steps 2 to 4 (center, mask, distance fit).
The bearings of the arms (+48 / +138 / -134 / -44 degrees) depend on how the holder is rotated relative to the
front of the toio, and the current mask is specific to this unit's mounting.

The holder is yellow on purpose: the base and arms reflected in the mirror are **masked by hue**
("Masking the holder by hue" in Step 3). If you print it in another color, adjust `--holder-hue` accordingly.

The pin assignment follows [m5stack/TimerCam-arduino](https://github.com/m5stack/TimerCam-arduino).

| Purpose | GPIO |
|---|---|
| Power hold (LDO hold) | 33 |
| LED | 2 |
| Battery voltage ADC | 38 |
| Camera XCLK / SIOD / SIOC | 27 / 25 / 23 |

**If GPIO33 is not held HIGH, the power cuts out when running on battery.** It is raised at the very start of `setup()`.

With the camera facing up and the convex mirror above it facing down, the surroundings appear in the image as a donut (annulus):
the area near the center is the reflection of the camera and holder base, and the outer rim corresponds to the horizontal direction (far away).

## Flashing the firmware

```bash
cd firmware/timercam_stream
cp include/wifi_config.h.example include/wifi_config.h
$EDITOR include/wifi_config.h        # leave WIFI_SSID empty to boot in AP mode
~/.platformio/penv/bin/pio run -t upload
~/.platformio/penv/bin/pio device monitor
```

The URL is printed on the serial port.

```
joining "..."........
STA  http://192.168.128.161/
     http://192.168.128.161:81/stream
```

If `WIFI_SSID` is empty or the join fails, it brings up an AP with SSID `TimerCamF-XXXX` /
password `omnivision` and becomes `http://192.168.4.1/`.
However, AP mode is not usable in practice (see "Choosing the connection mode" below).

Endpoints:

| URL | Content |
|---|---|
| `http://<ip>/` | Overlay UI for the browser (draws the mirror circles and the 72 rays over the image) |
| `http://<ip>/still` | A single JPEG |
| `http://<ip>/status` | JSON with the sensor settings and battery voltage |
| `http://<ip>/log` | The last 4KB of boot/runtime log (substitute for serial) |
| `http://<ip>/ctrl?var=saturation&val=2` | Change camera settings |
| `http://<ip>:81/stream` | MJPEG stream |

Only the stream is on port 81, so that the long-lived multipart response does not
block other requests (the standard esp32-camera arrangement).

## Choosing the connection mode (important)

**AP mode is unusable in a crowded 2.4GHz band.** Measured: even idle, with no stream
running, ping lost 33%; under load 72%; and eventually the association itself dropped. There are 28 2.4GHz APs
nearby, and changing the channel (interference score computed for every channel) did not help.

**Use STA mode (through an existing router).** If the PC is wired, the weak wireless hop disappears
from the path and things become dramatically stable. Set `WIFI_SSID` / `WIFI_PASS` in `wifi_config.h`
and it boots in STA mode.

If AP mode is the only option, widen the frame interval with `/ctrl?var=interval&val=<ms>`
to lower the throughput (default 150ms, about 6.7fps). This is a stopgap, not
a fix (measured: time to disconnect only grew from 7 s to 22 s).

## Calibration, end to end

Whenever the mirror, holder, or camera is moved, go through this from the top.

1. **Flash the calibration build**: set `START_WIFI_AT_BOOT` in `wifi_config.h` to 1 and flash.
   The measurement build (default 0) can bring up WiFi from BLE, but it cannot switch to JPEG due to lack of heap, so
   `/still` and `/stream` do not respond ("Known limitations").
2. **Center and radii** (Step 2): adjust `cx, cy, r_in, r_out` with `omni_viewer.py --set-framesize 8`
   and press `s`. Do not touch `theta0=90, ccw=true` ("Capture settings and bearing convention").
3. **Mask** (Step 3): with nothing on the floor, run `mask_editor.py --set-framesize 8 --holder-hue 15 40 --far-radius 139`,
   press `a` -> fix by hand -> `s`.
4. **Distance** (Step 4): place a target at a known distance and repeat `omni_range.py --record-distance <cm> --record-repeat 7`
   for each distance, then `fit_range.py --write --valid-range <min> <max>`.
5. **To the device** (Step 5): generate `scan_calib.h` and the ROS yaml with `gen_firmware_calib.py`,
   set `START_WIFI_AT_BOOT` back to 0 and flash.
6. **Verify**: with `ble_client.py --seconds 20`, check that an empty floor produces no hits and that an object at a known
   position appears at the correct bearing and distance.

## Checking the 360-degree view

```bash
pip install -r tools/requirements.txt
tools/omni_viewer.py --url http://<ip>:81/stream --set-framesize 8

# When the link is unstable: fetch /still N times per second instead of MJPEG
tools/omni_viewer.py --still-url http://<ip>/still --still-rate 4 --set-framesize 8
```

To cope with the stream silently stalling (socket stays open), it reconnects automatically if no frame
arrives for 4 seconds. The reconnect count is shown as `drops` in the raw window.

Three windows appear.

- **raw** — the raw image. Green circle = outer rim of the mirror, orange circle = camera reflection at the center, cyan lines = the 72 bearings at 5-degree steps.
  `360deg inside frame: YES/NO (xx%)` is shown at the top. **This is the verdict of the 360-degree check**;
  it says NO when the outer circle extends beyond the image (that direction is out of view).
- **panorama** — the annulus unwrapped as "horizontal = bearing / vertical = radius". Ticks every 30 degrees.
- **saturation** — the saturation channel of the panorama. A preview of what gets thresholded in Step 3.

Adjust `cx, cy, r_in, r_out, theta0` with the trackbars. Keys:

| Key | Action |
|---|---|
| `a` | Auto-estimate the outer rim of the mirror with a Hough transform (an initial guess only) |
| `s` | Save to `calib/mirror_calib.json` (loaded automatically on the next start) |
| `c` | Save raw / overlay / pano / sat as PNG |
| `q` | Quit |

`ccw` is the direction in which the bearing increases, `flip_r` swaps the panorama top and bottom (inner <-> outer).
A convex mirror mirrors the image left to right, so `ccw` is used to match the rotation direction in real space.

### Checking without hardware

```bash
tools/make_synthetic_omni.py -o /tmp/omni.png
tools/omni_viewer.py --source /tmp/omni.png
```

If the 12 colored sectors appear as vertical bands in order on the panorama, the unwrapping is correct.

## What to check (Step 1 checklist)

1. Does the outer circle fit inside the image (`360deg inside frame: YES`)?
   If it spills out, increase the mirror-to-camera distance or use a smaller mirror.
2. Is the center reflection (`r_in`) not too large? This becomes the blind zone.
3. Is the panorama continuous over 360 degrees without gaps (left and right edges join)?
4. Is the saturation difference between floor and objects visible in the **saturation** window?
   If it is hard to see, use `/ctrl?var=saturation&val=2` and lock AWB/AEC/AGC
   (`awb=0, aec=0, agc=0`) to stop exposure fluctuation. Step 3 assumes they are locked.

## Capture settings and bearing convention

Only the conclusions that keep holding after a mirror swap. Derivations and measured values are in [NOTES.md](NOTES.md).

### White balance is fixed at `wb_mode=1` (sunny)

Since detection is based on saturation, **color cast** is more fatal than blown highlights. All 5 presets were measured, and
sunny, where the floor saturation was the most neutral, was made the default (contrast 10.3x; auto was 6.8x).

**Do not use `awb=0` (AWB off).** On the OV3660 it does not hold the last automatic value but falls back to the default gains,
and the floor saturation worsens from 21 to 62. Locking a preset is the correct lever.

Hue is unstable in dark areas, so **gate on `V>80`** before judging by saturation.

### Bearing is `theta0=90, ccw=true`

With this, **panorama column angle = ROS bearing** (x forward, CCW positive) holds directly.
The camera mounting orientation has not changed, so these two have stayed the same across mirror swaps.

**Aligning objects in a still image alone does not determine the sign (`ccw`).** Move a known object by a known angle
and watch which way the image moves. If you fit a still image with the wrong `ccw`,
**a plausible solution comes out anyway** (this actually happened once: a wrong object correspondence led to the wrong conclusion
that the center needed a 13px correction).

### Use `/log` when there is no serial

A 4KB ring buffer in RAM; `logf()` writes to both serial and the buffer. A heartbeat with
uptime / battery voltage / heap / RSSI is logged every 60 seconds, so battery drain can be tracked as well.
Since the mirror mount blocks the reset button and unplugging USB also kills serial,
this can be the only window into the device.

## Step 3: boundary detection in 72 bearings

```bash
# 1. With nothing on the floor, create the ignore mask
tools/mask_editor.py --url http://<ip>:81/stream --set-framesize 8 \
    --holder-hue 15 40 --far-radius 139   # a to auto-generate -> fix by hand -> s to save

# 2. Scan
tools/omni_range.py --url http://<ip>:81/stream --set-framesize 8
tools/omni_range.py --source shot.png --once --json scan.json
```

`--set-framesize` is shared by the 3 tools (`omni_viewer` / `mask_editor` / `omni_range`).
**Let the tool that writes the calibration set it itself.** `mirror_calib.json` records the frame size
and is only valid at that size, so if the "command that starts calibration" and the "command that sets the size"
are separate, they will eventually drift apart.

For each bearing, the panorama column is scanned from the inside (near) to the outside (far),
and the first radius that departs from the floor is returned. In a convex mirror, **a tall object's base appears innermost**,
so the first hit of the outward scan is the nearest obstacle in that bearing.

The decision takes the max of 2 cues. Saturation alone is not enough:

- **Saturation**: chromatic objects are 6 to 10 times the floor. The strong cue.
- **Value**: gray and black objects barely move the saturation, so the deviation in value is also checked.

The threshold is not an absolute value but a **robust sigma from the floor** (MAD-based). The object being searched for
is itself an outlier, so with a standard deviation the object would hide itself. Saturation counts **upward only**
(floor shadows lower saturation, and this keeps them from being detected as objects).

The output is **a radius on the source image (px), not a distance**. Radius -> distance conversion is
Step 4 and needs the mirror shape calibrated with targets at known distances.

### Choosing the framesize (measured)

Two constraints collide: **the outer rim of the mirror must fit in the frame** (bearings that do not fit cannot be measured) and
**RGB565 capture must be fast** (this sets the scan rate).

Square modes such as `320x320` are **sensor crops** with a narrow field of view; only the 4:3 modes give the full field.
This was confirmed by correlating against XGA in JPEG (QVGA 0.995 / CIF 0.993 are full field, HVGA is a crop at 0.856).

`/bench?rgb=1` on the real device and fitting the outer rim of the stainless hemispherical mirror:

| framesize | Full field | ms/frame | Hz | `r_out` | Usable radius |
|---|---|---|---|---|---|
| 7 = 320x320 | x crop | 96.0 | 10.4 | 156 | 81 px, **75 degrees of bearing missing** |
| 6 = QVGA 320x240 | yes | 90.0 | 11.1 | 117 | 61 px |
| **8 = CIF 400x296** | yes | **112.6** | **8.9** | 146 | **76 px** |
| 9 = HVGA 480x320 | x crop | 126.9 | 7.9 | — | — |
| 10 = VGA 640x480 | yes | **450.2** | **2.2** | 234 | 122 px |

**VGA has 3 times the bytes but takes 4.7 times as long.** 450 is about 90x5: the sensor keeps producing 11 fps but
the driver completes only 1 frame in 5 (DMA bandwidth into PSRAM is saturated). Reducing `fb_count`
to 1 makes it worse at 585 ms, so there is no way out through settings.

**CIF was chosen.** It keeps 8.9 Hz with almost the same usable radius as 320x320, and no bearings are missing.
VGA if resolution is the priority, but that trades for 2.2 Hz.

`--set-framesize` works in all 3 of `omni_viewer.py` / `mask_editor.py` / `omni_range.py`.
Because the enum values shift with the library version, **the size of the received frame is checked against the expected one and a warning is issued**.

### The floor reference is taken from the inner half of the annulus

Near the outer rim the image shows walls, furniture, and ceiling, not floor. Mixing that into the reference statistics
inflates the spread and buries real objects. The effect of restricting the reference to the inner 50%:

| Reference range | Floor saturation MAD | Max deviation on an empty floor |
|---|---|---|
| Whole annulus | 13.3 | **4.6 sigma** (false detection)|
| Inner 50% | 11.9 | **2.9 sigma** |
| Inner 35% | 10.4 | 2.8 sigma |

This allowed the threshold to be lowered from 4.5 to 3.0, and detection of a blue bar 5cm ahead
**increased from 8 bearings to 14**. False detections on an empty floor stayed at 0.
(The numbers in the table were measured with the first convex mirror. The conclusion and defaults are still used as is in the current configuration.)

**With the hemispherical mirror there is, in addition, a permanent source of false detection on the outside.** The outer edge of the annulus (just inside the horizon)
sees walls and furniture and produces runs peaking at 3.6 to 4.4 against the threshold of 3.0. Since these straddle
`confirm`=4.0, the detection jumps outward the moment the target-side run dips below 4.0.
**Mask it out** ("The far end is set by the mask" in Step 4).

### Thresholds were set by measurement

Sweeping both over images with nothing on the floor (where anything is a false detection) and images with objects at known positions,
`threshold=3.0, min_run=8, val_threshold=6.0` were made the defaults.

**The key point is that saturation and value have separate thresholds.** Lowering the value threshold from 6.0 to 4.5
increases bar detections but gives 8 false detections on an empty floor (shadows reach 3.0 sigma).
The saturation side stays at zero false detections even down to 3.0.

### The robot's shadow is not masked

Measured:

| Region | Saturation S | Value V |
|---|---|---|
| Bright floor | 13.9 | 175.1 |
| Robot's shadow | 8.4 | 134.4 |

**The shadow does not move saturation at all (deviation 0.0 sigma) but moves value by 3.0 sigma.**
The shadow must not go into the static mask: when the robot moves the shadow direction changes too, so
a fixed mask would be wrong. Instead, the weaker cue, value, has its own threshold
(`--val-threshold`, default 6.0). That is why it is higher than the saturation one at 4.5.

### Ignored regions are held in a mask

Not as a per-bearing `r_in`, but as a mask in panorama coordinates. An occlusion like a cable, which
crosses multiple bearings at different radii, cannot be expressed as one radius per bearing.
The mask can be auto-generated with `a` (each column is scanned outward, and everything up to where it exceeds
72% of the floor value is treated as ignored), but **do it with nothing on the floor**.
If an object is present it gets masked along with everything else.

### Masking the holder by hue (2026-11-01, yellow holder)

Attaching the holder adds 3 things to the mirror image. All of them are as bright as the floor, so
the value-only auto-generation with `a` judges them entirely "usable" (97% stays valid),
and on the next scan all of them become obstacles. So when `--holder-hue LO HI` (OpenCV hue 0 to 179,
yellow is `15 40`) is given, `a` also paints by hue.

| What appears | Location (calibration px) | Treatment |
|---|---|---|
| Base around the lens (square) | 40 at the bearings of its sides, 56 to 64 at the corners and the roots of the arms | Per column, closed from the inside up to the last yellow row |
| The 4 arms | Bearings +43 to 57, +135 to 141, -140 to -128, -50 to -38 (90-degree spacing) | They block all radii, so the whole column is closed (3 degrees margin). 7 bearings permanently blind on the PC side, 12 on the device side |
| The outer ring | Outside the mirror rim (124 to 127 device px) | Already dropped by the far end at 139 |

**Arms are judged by the rise in saturation above the floor (+15), not by an absolute value.** The arm on the side away from the light source
has a saturation of only 44 and is 3 degrees wide, and an absolute threshold (S>=70) only picked up 3 of the 4.

**If yellow mixes into the floor reference, self-masking occurs.** If the square of the base stays in the valid region,
the floor saturation inflates to 37 to 74 (the real floor is 21), the threshold rises, and both the arms and the square
escape detection. A "exclude what was detected" procedure does not work in this state, so
the mask is built by hue first.

Measured after attaching the holder (CIF; only the center of the calibration re-tightened to within 2 px, `r_out` and the distance fit left as they were):
0 false detections on an empty floor (excluding the reflection of the USB cable at +145 to +170 degrees).
The mask discrepancy between 2 captures was 1.9%. The near end recedes from 2.2 cm to 3.6 cm.
The device side (BLE, 98 scans) is the same, with hits only at the cable at +155 to +170 degrees.

**The device mask was changed to "use only when the majority of the bearing cell's columns are open"** (`gen_firmware_calib.py`).
The PC zeroes masked columns when taking the median of 10 columns, so a bearing that is mostly masked outputs nothing, but
the device's 5 sub-rays have no mask and the median picks up the edge of an arm. With `any` as it was, a
5 cm obstacle appeared at +140 degrees in 14% of scans. This is why the blind bearings grew from 7 to 12.
`--far-radius 139` is there so that `a` closes the far-end band at the same time (table in Step 4).

`r_in` stays at 25.6. `gen_firmware_calib.py` says it could be raised to 27 device px, but
`range_fit.json` checks calibration identity through `calib_r_in`, so remeasure the distances before raising it.

## Step 4: radius -> distance (done, stainless hemisphere + CIF 400x296)

```bash
# For each distance: place the target d cm from the mirror axis and record the median of 7 frames
tools/omni_range.py --url http://<ip>:81/stream --set-framesize 8 \
    --record-distance 6 --record-bearing 0 --record-repeat 7   # --record-bar when the target is a straight bar
# Once all points are in, fit
tools/fit_range.py --write --valid-range 4.5 21
```

Distance is **the horizontal distance from the mirror axis (= lens center) to the near face of the target**. The model defines it that way,
so it is not from the front of the cube. A chromatic box or bar with a large saturation difference from the floor is the easiest target to handle.

`r(d) = -8.31 + 115.76 * arctan(d / 7.300)` (6 points / 4.5 to 21cm, **residual rms 0.9px, max 1.6px**)

| Radius (calibration px) | Distance | Resolution | |
|---|---|---|---|
| 70 | 5.9 cm | 0.10 cm/px | measured range |
| 90 | 8.3 cm | 0.14 cm/px | measured range |
| 110 | 11.9 cm | 0.23 cm/px | measured range |
| 130 | 18.5 cm | 0.47 cm/px | measured range |
| 139 (mask edge) | 24.1 cm | 0.75 cm/px | extrapolated |

**The LaserScan range is 0.036 to 0.233 m** (the near end is set by the reflection of the holder base; "Masking the holder by hue" in Step 3).

### The far end is set by the mask, not `r_out`

A hemispherical mirror's **rim looks above the horizon**, so there is no distance corresponding to `r_out`
(the horizon of this fit is at r=173.5; `r_out` is 200). Moreover, just inside the horizon
one px is tens of cm, so a number comes out but is unusable.

So **the outer band is dropped by the mask**, and `gen_firmware_calib.py` computes `range_min/max`
**from the radii where the mask is actually open**. Where to cut is decided by the acceptable resolution:

| Acceptable resolution | Far end | Radius to cut the mask |
|---|---|---|
| 0.50 cm/px | 19.2 cm | 132 calibration px |
| **0.75 cm/px** | **24.1 cm** | **139 calibration px** (adopted) |
| 1.00 cm/px | 28.1 cm | 144 calibration px |

Without cutting, using all the way to `r_out`, `range_max` is generated as **1.6 m**. That region is 41 cm/px, so
feeding it to nav2 would scatter meaningless points far away.

### `r_in` is not the blind zone

`r_in` only decides **where the 100 radius samples start**. The blind zone is held by the mask.
With the hemispherical mirror, self-reflection and occlusion vary by bearing from 0.13 to 0.45 `r_out`, so
one radius per bearing cannot express it.

**Put `r_in` at "the radius that can see closest in any bearing".** Set higher, the bearings that can see closer
are silently crushed; set lower, the samples before the opening are wasted.
`gen_firmware_calib.py` tallies the mask opening radius per bearing and points out any discrepancy.

For the same reason, the floor reference statistics are taken from **the inner 50% of each bearing's "usable range"**
(not the inner 50% of the radius index). Cutting by raw index leaves bearings with a wide blind zone
with zero reference samples, and the floor model gets built from only some bearings.
Both the device side (`scan.cpp`) and the PC side (`omni_range.py`) follow the same rule.

### Guards against mixing up calibration and measurement

This stage saw 3 mix-ups. All were hard to notice because "a plausible value comes out".
The tools now reject them.

| Accident | Guard |
|---|---|
| Used a distance fit for VGA/XGA with the 320x320 calibration | `range_fit.json` records `calib_r_in`/`r_out`, and no distance is output on mismatch |
| Measured 8 points in XGA while the calibration was still 320x320 | `mirror_calib.json` records `frame_width`/`height`, and a frame of a different resolution **stops with an error** |
| Recorded a value where the target was stuck to the mask boundary | Warns if within 8px of the mask start radius in that bearing |
| Samples from a different mirror or framesize got mixed into `range_samples.json` | **Stops with an error** if a sample's `frame_width` differs from the calibration (`record_sample` appends, so they accumulate) |
| Radius regressed or stalled against distance all the way to the end | Monotonicity is checked at every record; a change of less than 2 rows or a regression gives a warning |

**"The calibration does not carry over if you change the resolution"** is the key point. The square modes are not a downscale of 4:3 but
a sensor crop, so a simple scale cannot convert them
(320x320 is the central 768x768 of 4:3, cropped and downscaled: measured NCC 0.980 versus 0.644).

### Measurement procedure (for the next remeasurement)

1. **One frame is not a measurement.** Even with the target held still, the radius scatters with an sd of 2 to 3 calibration px,
   which is about the same as what moving 1 cm buys. Take the median with `--record-repeat 7`
   (median rather than mean, to discard the occasional frame that latches onto something else).
2. After changing the distance, confirm on the spot that the recorded radius **increases monotonically**.
   The tool warns, but also check that the slope **decreases monotonically**. The curve is compressive, so
   if the slope comes back up, the point shared by those 2 intervals is suspect.
3. **Take everything in one session.** Do not touch the robot, mirror, or camera. Fix the ruler to the floor.
4. Space the steps so that **the radii are evenly spaced**. Equal spacing in distance clumps the far points
   on the radius axis and leaves the steep near side thin. After 3 points, draw a provisional fit
   and reassign the remaining distances.
5. The first point is prone to "measured before moving". Compare it against the second.

**Drop outliers because they are "physically impossible", not because they "don't fit".** This time
the single point at 8 cm was excluded, and the reason was not only the residual of 5.3 px (others max 1.6 px) but
that including it makes the sequence of slopes non-monotonic. The remaining 6 points sit at rms 0.9 px.
Note that remeasuring 8 cm reproduced the same value (cause unresolved).

### Assigning radii to distances

```bash
# After 3 points, draw a provisional fit and back out the distances that give evenly spaced radii
tools/fit_range.py --write
python3 - <<'EOF'
import json, numpy as np
fit = json.load(open('calib/range_fit.json')); cal = json.load(open('calib/mirror_calib.json'))
a, b, c = fit['a'], fit['b'], fit['c']
for r in np.linspace(cal['r_in'] + 10, 139, 10):
    t = (r - a) / b
    print("r=%6.1f -> %s" % (r, "%.1f cm" % (c*np.tan(t)) if 0 < t < np.pi/2 else "past the horizon"))
EOF
```

## Step 5: on-device processing (done)

```bash
tools/gen_firmware_calib.py     # write calibration, mask, and fit into a C header
# Always rerun and flash after recalibrating (do not copy by hand)
curl http://<ip>/scan           # returns the 72 bearings as JSON
```

`omni_range.py` was ported to `firmware/.../scan.cpp`. The parameters are
generated into `scan_calib.h`, so they do not diverge from the PC side.

### Agreement with the PC side

Since `scan_calib.h` is generated from the same calibration, the device and the PC should give the same answer.
The median is offset between RGB565 and JPEG, but the detection is `(value - median)/MAD`, so it cancels out.

**The difference in the current configuration is 2.9 device px** (`/scan` gives 54.5 and the PC 51.6 on the same scene).
That is 2.6 steps of the device's radius sample spacing of 1.10 px, and quantization alone does not fully explain it.
In the 320x320 days they agreed to within -1.7 px, so this remains an **unresolved difference**.

Furthermore, continuous measurement over BLE scatters from 8.2 to 10.1 cm at bearing 0 (PC side: sd 0.9 device px over 9 frames).
Raw RGB565 noise and the fact that there are 5 sub-rays (the PC uses 10 columns) seem to contribute,
but this has not been isolated.

Comparing them requires the identical frame. The measurement build refuses `/still` due to lack of heap, so
**either temporarily flash the calibration build or add a path for `/scan` to dump the raw samples**.

### The camera LED must be turned off

`setup()` used to light the LED as a "running" indicator, but **the mirror reflects it into the annulus and it is detected as a strong,
highly saturated blue object**. On an empty floor, 3 phantom obstacles
(strength up to 7.9) appeared. Turning it off gave 0/72. Turning it back on reproduced the same 3 bearings.

`auto_mask` only picks up dark self-reflections, so **bright specular reflections are not
masked automatically**. The LED is now off in the initial state (`/ctrl?var=led&val=1` turns it on).

### Processing time and latency

| Stage | Time |
|---|---|
| capture (RGB565 CIF 400x296) | 113 ms |
| sample (72x100x5 points) | 43 ms |
| detect | 12 ms |
| **Age of the frame being measured** | **98 ms** |
| publish interval (`scan_period` 200 ms) | 216 ms |

Detection itself is light at 55 ms; capture is the bottleneck. sample and detect do not depend on resolution, so
only capture grows when the framesize is raised.

**Measure the frame age, do not estimate it.** The difference between `fb->timestamp` and the current time
is logged from the scan task every 2 seconds.

```
latency: frame age 98 ms, publish cycle 216 ms, scan 48 ms (11 scans)
```

#### `fb_count` is 1 in measurement mode

With `fb_count=2`, the driver fills both buffers and stops, and when the task wakes after sleeping `scan_period`
it is handed **the older one**. Measured:

| `fb_count` | Frame age |
|---|---|
| 2 | 234 to 300 ms |
| **1** | **90 to 101 ms** |

With 1 there is no queue, so `esp_camera_fb_get()` can only return the capture that completed most recently.
Capture no longer overlaps the 48 ms scan, but `scan_period` was throwing that
throughput away anyway, so there is no practical loss (4.1 Hz is unchanged).
The duplicate skip in the source ("32% of scans were reprocessing the same frame")
no longer triggers with `fb_count=1`.

#### Use the wrong clock and you measure clock drift while thinking you measure latency

esp32-camera stamps `fb->timestamp` with **`esp_timer_get_time()`**. Compare that against
`gettimeofday()` and the drift between the RTC and APB clocks masquerades as latency.
It actually read **730 ms**, and "improved" to **555 ms** after reflashing — only because the uptime
was reset and less drift had accumulated. Compared on the same clock, it is 98 ms.

### BLE output (verified)

```bash
tools/ble_client.py --seconds 15        # receive distances only
curl "http://<ip>/radio?wifi=0"         # stop WiFi and run BLE alone
```

Measured: **69 scans / 17.1 seconds = 4.0 Hz, 0 dropped packets, 606 B/s**.
Getting the same result over the JPEG stream took about 200 kB/s, so this is **1/330 of that**.

The packet is 150 bytes (magic, version, n_rays, seq, `u16 dist_mm[72]`,
0xFFFF = no detection). The bearing of ray i is i x 5 degrees, which is directly the ROS bearing.

**Use NimBLE.** Bluedroid does not have enough heap when it coexists with the camera + WiFi, and
`BLEDevice::init()` never returns. With NimBLE, Flash went 58%->42% and RAM 32%->30%.
Also, **1.4.x targets arduino-esp32 2.x**; on 3.x, `NimBLEDevice::init()`
calls `abort()`. Use the 2.x series (`setScanResponse` was renamed to `enableScanResponse`).

**The scan task needs an 8 kB stack.** `scan_frame` put a 2 kB histogram +
1 kB on the stack, and with a 4 kB task the whole chip reset on the first BLE
connection (over HTTP this did not surface because the httpd stack had been set to 8 kB).
The histogram was moved to static.

### Measured power consumption

Measuring discharge requires unplugging USB, which also stops serial and charging.
WiFi is stopped too, so **BLE is the only remaining window**. Battery voltage and uptime are
bundled into the scan packet (not a separate characteristic
— with a stale BlueZ GATT cache the new attribute is invisible, and tracking that down
cost time).

```bash
curl "http://<ip>/radio?wifi=0"          # stop WiFi (before unplugging USB)
# unplug the USB cable here
tools/ble_client.py --seconds 1800 --battery calib/discharge_ble.csv --battery-period 60
tools/analyse_discharge.py calib/discharge_ble.csv
```

**Measured (BLE alone, scanning at 11.6 Hz):**

| Item | Value |
|---|---|
| Discharge | **413 mV/h** |
| Voltage band | 4133 -> 3961 mV (0.42 h) |
| Estimated runtime | **about 1.9 hours** (4.2->3.4 V) |

`/radio?wifi=0` is runtime only; WiFi comes back on reboot. The watchdog does not
reboot when it was stopped intentionally.

**Stopping WiFi raises BLE throughput** (4.0 -> 11.3 Hz). The time-sharing of the
single radio shows up directly.

### BLE only at boot (default)

With `START_WIFI_AT_BOOT 0` (default) in `wifi_config.h`, **only BLE comes up at power-on**.
WiFi is the main consumer, and since measurement is completed on the device it is not needed.

When HTTP is needed for calibration or debugging, bring it up from BLE:

```bash
tools/ble_client.py --cmd wifi-on     # WiFi + HTTP server come up
```

**The HTTP server must not start before WiFi.** `httpd_start()` creates sockets,
so if the network stack is not initialized it hits an
`xQueueSemaphoreTake` assert inside FreeRTOS and boot-loops. Call `server_start()`
only after `wifi_start()` (double starting is guarded too).

### Power-off and reboot (assuming the reset button is blocked)

Since the mirror mount makes the reset button unreachable, reboot and power-off
are available from both BLE and HTTP.

```bash
tools/ble_client.py --cmd reboot                    # reboot
tools/ble_client.py --cmd off                       # power off (battery only)
tools/ble_client.py --cmd scan-period --scan-period 500
curl "http://<ip>/power?reboot=1"                   # when WiFi is available
curl "http://<ip>/power?off=1"
```

**Power-off works by releasing the LDO hold on GPIO33**, so **it does not cut off while USB is plugged in**
(the rail keeps being powered from USB). In that case the log shows
`still alive: USB is powering the rail`.

Commands are received as **writes to the scan characteristic**. When a dedicated characteristic
was added it was invisible because of BlueZ's stale GATT cache, and clearing the cache did not work
due to permissions, so it was moved to a path that reliably works.

**On reboot, WiFi comes back and `scan_period` also returns to its default (200ms).**
To change it permanently, change the default in the firmware.

### Scan frequency is the main knob for power consumption

One scan is about 52 ms of CPU. The sensor produces about 11 fps, so left alone
the core is nearly saturated. `scan_period` (default 200 ms = 5 Hz) thins it out.

```bash
curl "http://<ip>/ctrl?var=scan_period&val=200"   # 0 = no cap
```

With a 23 cm range at 5 Hz, an obstacle is seen 5 times even when driving at 20 cm/s.
Raise it if you drive faster.

**Duplicate frame removal was a miss.** Comparing the bench's 7.9 fps with the actual 11.6 Hz,
it was guessed that "32% is reprocessing of the same frame", but adding a timestamp check
left the rate at 11.0 Hz — there were almost no duplicates. An example of mistaking a guess for a
measurement. The check itself is harmless, so it stays.

## Step 6: output to ROS 2 (verified on real hardware)

`ros2/toio_omni_range` publishes the BLE scans as `sensor_msgs/LaserScan`.
**Making it a LaserScan is the key point**: RViz, costmaps, and scan matching
then work without knowing anything about this sensor.

```bash
docker/build-docker-image.bash            # first time only
docker/run-docker-container.bash
# inside the container
colcon build --symlink-install && source install/setup.bash
ros2 launch toio_omni_range omni_range.launch.py
ros2 launch toio_omni_range omni_range.launch.py device_address:=AA:BB:CC:DD:EE:FF
```

| topic | Type | Content |
|---|---|---|
| `scan` | `sensor_msgs/LaserScan` | 72 bearings, 5-degree steps |
| `omni/battery` | `sensor_msgs/BatteryState` | vbat (percentage is NaN since it is not fitted)|
| `/diagnostics` | `DiagnosticArray` | Rate, gaps, reconnects, out-of-range counts |

There are 2 differences from a spinning LiDAR.

- **A scan is a snapshot of one camera frame.** All 72 rays share the same instant, so
  there is no distortion during the sweep and `time_increment` is 0.
- **It reaches about 23 cm, and the near 3.6 cm is a blind zone.** `range_min` / `range_max` are generated by
  `gen_firmware_calib.py` into `params/omni_range_calib.yaml` from the same calibration as the firmware
  (0.036 to 0.233 m). **Do not write them by hand** — a LaserScan that disagrees with the mirror
  makes downstream silently discard every point.

### Running with nav2

The procedure for bringing it up together with the cube is in [ros2/toio_omni_bringup/README.md](ros2/toio_omni_bringup/README.md), and
map building in [ros2/toio_omni_mapping/README.md](ros2/toio_omni_mapping/README.md). Just the essentials:

```bash
# inside the container (toio_ros2 must be in the same workspace)
ros2 launch toio_omni_bringup toio_omni_bringup.launch.py \
    cube_id:=a1b device_address:=AA:BB:CC:DD:EE:FF          # cube + sensor + RViz
ros2 launch toio_omni_mapping mapping.launch.py \
    cube_id:=a1b device_address:=AA:BB:CC:DD:EE:FF          # drive around the mat and build a map
ros2 launch toio_omni_bringup toio_omni_nav2.launch.py \
    cube_id:=a1b device_address:=AA:BB:CC:DD:EE:FF          # nav2. Set the goal with 2D Goal Pose in RViz
```

- **Specify both `cube_id` and `device_address`.** The cube and the sensor use the same BLE adapter,
  so running a discovery scan destabilizes the connections sharing it.
- The 2 drivers know nothing of each other and only meet through TF (`map -> center -> omni_link`).
  The mirror position is `sensor_x/y/z/yaw`, and `sensor_yaw=0` (sensor 0 degrees = cube front) has been verified.
- Commands flow `controller_server -> cmd_vel_raw -> collision_monitor -> cmd_vel -> toio_ros2`.
  `goal_pose` is diverted to `cube_goal_pose` (the cube's built-in target motion does not go through `cmd_vel`
  and conflicts with nav2).
- Walls must be **a saturated color** and taller than the mirror. A white wall on a white floor does not show.
- **Top-heavy, so speeds are set by not falling over.** nav2 uses 0.03 m/s and 2.0 rad/s,
  the cube's built-in target motion `goal_max_speed:=15`, teleop `-p speed:=0.03 -p turn:=1.8`.
  nav2's default Behavior Tree backs up 0.30 m at 0.15 m/s during recovery, so the package carries a BT
  with BackUp changed to 3 cm at 0.025 m/s. Lowering only one of these still tips it over
  ("Top-heavy: speed is set by stability" in the bringup README).

### The array needs reordering

Since `bearing(i) = ((i*5 + 180) mod 360) - 180`, the wire order is not monotonic in bearing
(ray 36 is -180 degrees). LaserScan requires ascending bearing, so
the driver reorders with `omni_protocol.BEARING_ORDER`.

**Get this wrong and a scan rotated by 180 degrees is displayed "plausibly".** That is why
no constant rotation amount is written and the reordering is derived from the bearings.
`test_omni_protocol.py` verifies the correspondence between the reordering and the angles.

### Contention for the BLE adapter

The toio cube (toio-py -> bleak) and the sensor (bleak) use the same HCI. A discovery scan
for connecting destabilizes the connections sharing it, so **pin the address once found
in `device_address` and avoid scanning**. The node logs the address it found at startup.
If that is still not enough, split onto a dongle with `ble_adapter:=hci1`.

### `BleakClient(timeout=)` is not a connection timeout

bleak's `BleakClient(address, timeout=...)` only applies to the **implicit discover when an address is given**;
`connect()` itself waits indefinitely on BlueZ. If the previous connection did not
close cleanly the unit stops advertising, and at that point it
**hangs forever with no error and no log**.

Always wrap it in `asyncio.wait_for(client.connect(), timeout=...)`.
`toio_ros2` hit the same trap and fixed it.

What makes this nasty is **how it looks downstream**: scans simply stop arriving, so
nav2's collision_monitor decides "sensor dead" and halts the robot, and
all you see is "nav2 is not issuing commands".

### Unverified

Driving around obstacles under nav2 has been verified (the video at the top; the configuration before the mirror holder).
nav2 driving in the current configuration with the holder, and **the behavior of connection, reconnection, and scan dropouts, have not been measured**.
`obstacle_max_range` etc. in `nav2_params.yaml` were rewritten to match the current 0.233 m, but
it has not been driven with those values.

Remaining known discrepancies:

- **Device and PC differ by 2.9 device px.** On the same scene `/scan` gives 54.5 and
  the PC's `omni_range.py` 51.6. That is 2.6 steps of the device's radius sample spacing of 1.10 px.
- **Continuous measurement over BLE scatters from 8.2 to 10.1 cm at bearing 0** (PC side: sd 0.9 device px over 9 frames).
  Raw RGB565 noise and the 5 sub-rays (the PC uses 10 columns) seem to contribute, but this has not been isolated.
- **Only the 8 cm calibration point is off the curve by 5.3 px.** Reproduces on remeasurement. Cause unknown.

## Pitfalls (all hit on real hardware)

- **`/still` and `/scan` were returning a frame one request old.** With `grab_mode` set to
  `CAMERA_GRAB_WHEN_EMPTY`, the driver captures the next frame and fills the buffer the moment it is returned,
  so the next one-shot request receives "the frame captured at the time of the previous request".
  The stream keeps frames flowing, so you do not notice. **During calibration this silently pairs the "known distance"
  with "the radius of the previous target position".** Frames at 5 cm and 6 cm actually
  had images equivalent to 10 cm and 11 cm, and finding the cause burned several sessions.
  Fixed by discarding `fb_count` frames before grabbing in `camera_fb_get_fresh()`
  (can be verified by hitting a one-shot `/still` right after changing framesize and checking that the requested size comes back).
  The PC side has the same property, so `first_frame_at()` discards until a frame of the expected size arrives.

- **Flashing baud rate**: the Timer Camera F's FT232 stops responding when switched to
  460800 / 230400 after the stub flasher starts (`The chip stopped responding`).
  `upload_speed = 115200` is stable (about 60 seconds for 1MB).
- **The `framesize` enum values depend on the version.** In the esp32-camera bundled with arduino-esp32 3.3.7,
  `FRAMESIZE_VGA = 10` (older versions: 8). Check `sensor.h`.
- **The frame buffer is allocated at the size given to `esp_camera_init`**, so
  `set_framesize` cannot go larger than the initialization size. It initializes at UXGA and then
  drops to VGA.
- `/still` returns the old buffer for 2 or 3 frames right after a change. Discard a few frames after changing resolution.
- `CAMERA_GRAB_LATEST` keeps capturing even with no client and keeps printing `cam_hal: FB-OVF`,
  so `CAMERA_GRAB_WHEN_EMPTY` is used.
- When you cannot tell which M5Stack device is which over USB: it shows up as an FTDI
  `0403:6001 / Hades2001 / M5stack`. **Replug it and watch for the update in `stat -c %y /sys/bus/usb/devices/<n-m>`**;
  that is the reliable way.
- **Opening the port with pyserial resets the device.** DTR/RTS are asserted by default and
  the M5Stack's auto-reset circuit kicks in. Unless you explicitly call
  `setDTR(False)/setRTS(False)` right after opening, it stays held in reset and
  looks as if "WiFi died" (misdiagnosed once because of this).
- **Do not take large local arrays in `esp_http_server` handlers.** The task stack
  defaults to 4096 bytes, and a 4KB local buffer hangs the request.
  Raise `config.stack_size` or make it static.

## Known limitations

- Being a plain ESP32, it does about 10.5 fps at VGA with JPEG quality 12 (12 kB/frame, 126 kB/s,
  measured over softAP). It drops as the resolution goes up.
- **RGB565 tops out at VGA** (450 ms/frame = 2.2 Hz). This is why CIF is chosen for measurement.
- **Camera, WiFi, and BLE cannot all three run at once.** Any 2 can. Bringing up NimBLE
  with camera + WiFi running aborts with `ESP_ERR_NO_MEM` and boot-loops.
  Also, bringing up WiFi drops the internal heap to about 15 kB and `esp_camera_init()`
  **fails regardless of size or format** (even JPEG VGA cut down to `fb_count=1` does not work).
  So the calibration build (`START_WIFI_AT_BOOT 1`) runs JPEG + WiFi from boot without bringing up BLE,
  and does no mode switching at all. `camera_set_mode()` **refuses before deinit** if there is not enough heap
  (so that a failed switch does not take down the mode that was running).
- Consumption is high on battery. For regular use as an omnidirectional sensor, USB power is recommended.
- **The center of gravity is high.** With the mirror stacked on a 32 mm cube, it falls over on hard starts, hard stops, and turning in place.
  The driving speed is kept to about 3 cm/s (`toio_omni_bringup`). Because of the deadband, it cannot start with a motor value below 8,
  and that is the minimum jolt.
- The radius -> distance mapping of the unwrapping depends on the mirror shape. Calibrate with targets at known distances
  in Step 4. **A hemispherical mirror's rim looks above the horizon**, so the outside of the annulus has no distance.

## Verified environment and measured values

- PlatformIO Core 6.2.0 / espressif32 (pioarduino) 55.3.37 / Arduino ESP32 3.3.7
  -> Flash 42.6% (1,340,953 / 3,145,728 B), RAM 31.5% (2026-11-01, measurement build)
- Hardware: ESP32-D0WDQ6-V3 rev3.1 / Flash 4MB / PSRAM 4MB / MAC 3c:8a:1f:d7:84:4c
- Verified working: `/` `/still` `/status` `/ctrl` `/stream` `/scan` `/bench` `/radio` `/power`,
  framesize switching (QVGA / 320x320 / CIF / HVGA / VGA / XGA / UXGA),
  stream reconnection, BLE scan delivery, ROS 2 LaserScan -> RViz
- Mirror: uxcell stainless-steel hollow hemisphere, about 60 mm (replaced three times: convex mirror -> cone -> hemisphere.
  Why the cone was abandoned is described below)

## License

Apache License 2.0 ([LICENSE](LICENSE)). Aligned with `toio_ros2`, which it lives alongside.
