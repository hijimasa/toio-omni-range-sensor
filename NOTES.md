[English](NOTES.md) | [日本語](NOTES.ja.md)

# NOTES — History, and measurements that no longer apply

The README is limited to describing the current configuration (hemispherical stainless-steel
mirror + CIF 400x296). This file holds **the measurements that led there and the record of
options not adopted**. Most of the numbers are from the previous mirror and the previous
resolution, so they do not apply directly to the current setup. They are kept so the same
investigation does not have to be redone from scratch.

## Measurements on the real device (2026-08-08, **first convex mirror**)

> The numbers in this section are from the first convex mirror and the VGA calibration. The
> mirror has since been replaced with the hemispherical stainless-steel one, so the geometry
> values (`cx` etc.) are not current. The conclusions about saturation, white balance and
> reflections still hold.


Calibration (VGA px): `cx=263, cy=240, r_in=70, r_out=200`.
The outer circle fits entirely within the frame, **coverage 100%**.

Shots taken with a plain white floor as reference, with a blue box and a dry-cell battery placed on it:

Re-measured after the calibration was finalized, with `wb_mode=1` / ROS bearings:

| Target | Bearing | Max saturation S | Mean S over object | Mean V over object | Pixels |
|---|---|---|---|---|---|
| Floor (reference) | — | 67 | 61 | 176 | 45 |
| Blue tweezers | −90° | 114 | 79.3 | 125.2 | 1162 |
| Blue battery | +75° | **123** | 80.8 | **150.7** | **153** |

**Saturation comes out at roughly 2x the floor, and saturation itself does not drop for small objects.**
The battery's saturation looked low not because of shadow but because **the bearing mapping
was wrong and a different (empty) sector was being measured**. In fact its value (brightness)
is also higher than the tweezers'.

What matters is not the saturation value but **area**. The battery occupies only 18–26% of the
annulus radius, 16px × 15 columns = 153 pixels (the tweezers: 1162 pixels).
Distant and small objects have fewer pixels, so the thresholding in step 3
**must not discard small-area detections**.

### Reflection of the camera itself (blind zone)

Scan outward from the center and measure the radius where the reflection ends, in all 72 directions.
**The threshold must be relative to the floor value (brightness) of each image** (with a fixed
value, a mere change in exposure looks like a "regression". This was actually misread once).

Before and after unplugging the USB cable and switching to battery power:

| | Median | Worst | Directions >40% |
|---|---|---|---|
| With USB cable | 26% | 86% | 23/72 |
| Battery powered | **24%** | **58%** | **9/72** |

**All 14 directions that changed significantly improved** (zero regressions). The improvement is
concentrated in +45 to +115°, which is where the cable was reflected.

However, the worst direction still takes up 58% of the radius. The current `r_in=70` (= 35% of `r_out`) is
**too small in some directions**. In step 3, do not make `r_in` a constant;
the right approach is to **measure an inner-radius profile for all 72 directions once and keep it**.

### Battery operation

Unplugging USB does not cause a reboot, and WiFi is maintained (ping loss 0%, RSSI -40).
Drain can be tracked with the `/log` heartbeat. Measured (VGA idle, no stream):

```
up    180s  vbat 4.20 V     <- USB powered up to here
up    240s  vbat 4.12 V     <- USB unplugged
up    360s  vbat 4.09 V
```

### Bearing calibration (done)

`cx=263, cy=240, r_in=70, r_out=200, theta0=90, ccw=true` (VGA px).
With this, **panorama column angle = ROS bearing** (x forward, CCW positive) holds directly.

The geometry values are for the first mirror, but **`theta0=90, ccw=true` still apply as is**
(the camera mounting orientation has not changed). The "move it and check" procedure below is
needed every time the mirror is replaced.

**Aligning object positions in a still image alone does not determine the sign (`ccw`).**
The reliable way is to move a known object by a known angle and see which way the image moves.
Settled by moving the tweezers from 0° → −90°:

| Object | Ground truth | Detected | Residual |
|---|---|---|---|
| Tweezers (before move) | 0° | −3.0° | 3.0° |
| Battery | +75° | +74.8° | 0.2° |
| Tweezers (after move) | −90° | −90.0° | 0.0° |

Assuming `ccw=false`, the same data gives residuals of up to 170°. In other words,
**fitting a still image with the wrong sign yields a plausible-looking solution**
(this actually happened once: a wrong object correspondence led to the wrong conclusion that the center should be corrected by 13px).
Always verify by "moving it and checking".

### White balance (important)

Since detection is based on saturation, **color cast** is more fatal than blown-out highlights. Measured:

| wb_mode | Floor saturation | Floor hue | Object saturation | Contrast |
|---|---|---|---|---|
| auto | 18.8 | 226° | 128 | 6.8x |
| **1 = sunny** | **11.1** | 206° | 114 | **10.3x** |
| 2 = cloudy | 13.1 | 60° | 105 | 8.0x |
| 3 = office | 52.7 | 214° | 137 | 2.6x |
| 4 = home | 89.4 | 212° | 156 | 1.7x |

`wb_mode=1` was made the default. **Never use `awb=0` (AWB off)** ——
on the OV3660 it does not hold the last automatic value but reverts to the default gains, and floor saturation worsens from 21 → 62.

Dimming the lighting to eliminate shadows is correct. Dimming with auto WB makes things worse
due to a blue cast (floor hue 158° → 222°), but with sunny fixed it is
better (10.3x) than strong lighting + auto (5.9x).

Hue is unstable in dark regions, so **gate on `V>80`** before judging by saturation.

## USB cable, blind zone, and debugging means

What appears in the mirror as a "pillar" is the USB cable. However,
**this may be solvable without moving to battery power.**
The reflection of the camera itself occupies 19–26% of `r_out` in every direction,
and that is a blind zone anyway. If the cable is routed **straight down from the camera body
(along the optical axis)** instead of radially, it hides in that existing blind zone and the
additional blind zone can be made zero.
This recovers 255–285° while keeping USB power and serial, so it is worth trying first.

**Be careful about adding BLE for debugging.** On the ESP32, WiFi and BLE time-share the same
2.4GHz radio. WiFi in this configuration is already marginal (AP mode has 33% ping loss even
when idle; STA drops to 0.5fps when pushing bandwidth), and keeping BLE resident alongside it
could break the core function for the sake of debugging. If it is added, run it exclusively:
stop WiFi and switch to a battery-diagnostics-only mode.

## Why the cone mirror was not adopted

Along the way, a cone mirror was considered because "the convex mirror has a narrow measurable range". **The cone itself works**
(the depression angle is linear 1:1 with image radius, and bearing is preserved). When one was actually built and photographed,
neither the mirror surface quality nor the full-circle coverage was a problem. It was not adopted because **it did not fit in the frame at the measurement resolution**
and **it could not image the surroundings accurately**.

**Substituting a hexagonal pyramid is not acceptable.** The normal of a flat facet does not rotate with bearing, so
bearings get pulled toward the facet center. The compression ratio follows exactly from solid-angle conservation as `c = sinθ / cos δ`
(θ = angle from the optical axis, δ = depression angle after reflection). Confirmed by ray tracing:

| Half-apex angle | Facets | Coverage of 4–15cm band | Max blind zone |
|---|---|---|---|
| 65° | 6 | 20.0% | 48° |
| 75° | 6 | 60.0% | 24° |
| 75° | 24 | 60.0% | 6° |
| Any | Cone | **100%** | 0° |

**Increasing the facet count does not raise coverage** (the holes just become finer and more spread out). The holes are larger at close range.
To make a cone, the right way is to **roll up a sector (angle = 360°×sinα)** rather than cutting a hexagonal pyramid.
A plastic mirror is a developable surface so it bends, and only one seam is needed.

- Python 3.10 / OpenCV 4.5.4 / NumPy 1.26
