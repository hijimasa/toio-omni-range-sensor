[English](README.md) | [日本語](README.ja.md)

# docs — overview slides

**Generation scripts** for slides that explain this sensor on a single page. Two variants:
the **simple version** (large text, minimal numbers) and the **detailed version** (with
measured values and a breakdown of the processing).

**The generated files (pptx / pdf / png / figs\*.png) are not kept in the repository.** They
can all be rebuilt with the commands below, and they go stale every time the mirror or the
calibration changes. In fact, a version from the first convex-mirror days was still sitting
around saying "measurable range 4 to 11 cm".

| File | Contents |
|---|---|
| `make_simple.py` | Simple version. Generates figures and slides in one go |
| `make_figs.py` | Figures for the detailed version (`figs/hw.png` hardware layout, `figs/unwrap.png` ring to panorama, `figs/scan.png` output) |
| `make_slide.py` | Slides for the detailed version. Run after `make_figs.py` |

## Rebuild

```bash
cd docs
python3 make_simple.py            # figs/*_simple.png + toio_omni_range_simple.pptx
python3 make_figs.py && python3 make_slide.py   # detailed version
```

Requirements: `matplotlib`, `python-pptx`, a Japanese font.

## Fixing the numbers

The range, rate and processing time are **hard-coded** in the slides. After recalibrating,
check them against "Current configuration" in the README at the repository root and fix the
following.

| Location | Contents |
|---|---|
| `specs` in `make_slide.py` | Range, Hz, processing time per scan, battery life |
| `tiles` in `make_simple.py` | Measurable range |
| `set_yticklabels` in `make_simple.py` / `make_figs.py` | Radius ticks on the output figures |
