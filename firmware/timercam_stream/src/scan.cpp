#include "scan.h"

#include <math.h>
#include <string.h>

#include <esp_timer.h>

// Saturation and value samples for every (ray, radial) cell. Static rather than
// on the stack: 14 kB would blow the httpd task stack, and allocating per scan
// would fragment the heap at 8 Hz.
static uint8_t g_s[SCAN_N_RAYS][SCAN_RADIAL_N];
static uint8_t g_v[SCAN_N_RAYS][SCAN_RADIAL_N];

static inline bool mask_usable(int ray, int j) {
    return (SCAN_MASK[ray][j >> 3] >> (7 - (j & 7))) & 1;
}

// OpenCV's definition, straight from RGB: V = max, S = (V-min)/V * 255.
static inline void rgb565_sv(uint16_t px, uint8_t* s, uint8_t* v) {
    // The camera hands over big-endian halfwords.
    uint16_t p = (uint16_t)((px >> 8) | (px << 8));
    uint8_t r = (uint8_t)(((p >> 11) & 0x1F) * 255 / 31);
    uint8_t g = (uint8_t)(((p >> 5) & 0x3F) * 255 / 63);
    uint8_t b = (uint8_t)((p & 0x1F) * 255 / 31);
    uint8_t mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
    uint8_t mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
    *v = mx;
    *s = mx ? (uint8_t)((uint32_t)(mx - mn) * 255 / mx) : 0;
}

static uint8_t median5(uint8_t* a) {
    for (int i = 1; i < 5; ++i) {          // insertion sort, n = 5
        uint8_t k = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > k) { a[j + 1] = a[j]; --j; }
        a[j + 1] = k;
    }
    return a[2];
}

// Median and MAD from a 256-bin histogram. Exact for uint8 data and far cheaper
// than sorting ~7000 samples. MAD, not standard deviation: the objects being
// looked for are outliers and would inflate a std into hiding themselves.
static void robust_stats(const uint32_t* hist, uint32_t n, float* med, float* mad) {
    // static, not automatic: 1 kB here plus 2 kB of histograms in scan_frame
    // overflowed a 4 kB task stack and reset the chip on the first BLE connect.
    static uint32_t dev[256];
    uint32_t acc = 0, half = n / 2;
    int m = 0;
    for (; m < 256; ++m) { acc += hist[m]; if (acc > half) break; }
    if (m > 255) m = 255;
    *med = (float)m;

    memset(dev, 0, sizeof(dev));
    for (int i = 0; i < 256; ++i) {
        if (!hist[i]) continue;
        int d = i - m; if (d < 0) d = -d;
        dev[d] += hist[i];
    }
    acc = 0;
    int k = 0;
    for (; k < 256; ++k) { acc += dev[k]; if (acc > half) break; }
    if (k > 255) k = 255;
    *mad = (float)k * 1.4826f;
}

bool scan_frame(const camera_fb_t* fb, ScanResult* out) {
    if (!fb || !out) return false;
    if (fb->format != PIXFORMAT_RGB565) return false;
    if ((int)fb->width != SCAN_FRAME_W || (int)fb->height != SCAN_FRAME_H) return false;

    const uint16_t* px = (const uint16_t*)fb->buf;
    const int W = SCAN_FRAME_W, H = SCAN_FRAME_H;
    const float dr = (SCAN_R_OUT - SCAN_R_IN) / (SCAN_RADIAL_N - 1);
    const float sign = SCAN_CCW ? -1.0f : 1.0f;
    // Sub-rays spread across the direction's 5 deg cell, matching the PC taking
    // the median of the 10 panorama columns that belong to it.
    const float cell = 360.0f / SCAN_N_RAYS;

    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < SCAN_N_RAYS; ++i) {
        uint8_t sbuf[SCAN_SUBRAYS], vbuf[SCAN_SUBRAYS];
        float cs[SCAN_SUBRAYS], sn[SCAN_SUBRAYS];
        for (int k = 0; k < SCAN_SUBRAYS; ++k) {
            float frac = (SCAN_SUBRAYS == 1) ? 0.5f : (float)k / (SCAN_SUBRAYS - 1);
            float a = (SCAN_THETA0_DEG + sign * (i * cell + frac * cell)) * (float)M_PI / 180.0f;
            cs[k] = cosf(a);
            sn[k] = sinf(a);
        }
        for (int j = 0; j < SCAN_RADIAL_N; ++j) {
            float r = SCAN_R_IN + j * dr;
            for (int k = 0; k < SCAN_SUBRAYS; ++k) {
                int x = (int)(SCAN_CX + r * cs[k] + 0.5f);
                int y = (int)(SCAN_CY + r * sn[k] + 0.5f);
                if (x < 0 || x >= W || y < 0 || y >= H) { sbuf[k] = 0; vbuf[k] = 0; continue; }
                rgb565_sv(px[y * W + x], &sbuf[k], &vbuf[k]);
            }
            g_s[i][j] = median5(sbuf);
            g_v[i][j] = median5(vbuf);
        }
    }
    out->us_sample = (uint32_t)(esp_timer_get_time() - t0);

    t0 = esp_timer_get_time();
    // Floor reference from the inner slice only: the outer rows look past the
    // edge of the floor at walls and furniture, and letting those in inflates
    // the spread until real objects stop standing out.
    //
    // "Inner" is counted per ray over that ray's *usable* samples, not over the
    // raw radial index. The dead zone the mirror makes of the unit itself is
    // not one radius: measured on the hemispherical mirror it runs from 39 % to
    // 81 % of r_out depending on bearing. Slicing the raw index would hand the
    // reference nothing at all in the worst directions and the whole near half
    // in the best ones, so the floor model would be built from a biased handful
    // of bearings. Per-ray, every direction contributes its own near field.
    static uint32_t hs[256], hv[256];
    memset(hs, 0, sizeof(hs));
    memset(hv, 0, sizeof(hv));
    uint32_t n = 0;
    for (int i = 0; i < SCAN_N_RAYS; ++i) {
        int usable = 0;
        for (int j = 0; j < SCAN_RADIAL_N; ++j) if (mask_usable(i, j)) ++usable;
        int take = (int)(usable * SCAN_REF_FRAC + 0.5f);
        if (take < 1) continue;                 // fully masked bearing
        for (int j = 0; j < SCAN_RADIAL_N && take; ++j) {
            if (!mask_usable(i, j)) continue;
            hs[g_s[i][j]]++; hv[g_v[i][j]]++; ++n; --take;
        }
    }
    if (!n) return false;
    robust_stats(hs, n, &out->floor_s, &out->floor_s_mad);
    robust_stats(hv, n, &out->floor_v, &out->floor_v_mad);
    float smad = out->floor_s_mad < 2.0f ? 2.0f : out->floor_s_mad;
    float vmad = out->floor_v_mad < 2.0f ? 2.0f : out->floor_v_mad;

    out->hits = 0;
    for (int i = 0; i < SCAN_N_RAYS; ++i) {
        static float dev[SCAN_RADIAL_N];
        for (int j = 0; j < SCAN_RADIAL_N; ++j) {
            if (!mask_usable(i, j)) { dev[j] = 0.0f; continue; }
            // Saturation counts upward only: a floor shadow is *less* saturated
            // and must not read as an object. Brightness gets its own, higher
            // bar because the robot's own shadow already reaches ~3 sigma there.
            float ds = (g_s[i][j] - out->floor_s) / smad;
            if (ds < 0) ds = 0;
            float dv = fabsf(g_v[i][j] - out->floor_v) / vmad;
            float a = ds / SCAN_THRESHOLD, b = dv / SCAN_VAL_THRESHOLD;
            dev[j] = (a > b ? a : b) * SCAN_THRESHOLD;
        }
        // 5-tap moving average along the radius, standing in for the PC's
        // Gaussian blur; directions stay independent.
        static float sm[SCAN_RADIAL_N];
        for (int j = 0; j < SCAN_RADIAL_N; ++j) {
            float acc = 0; int cnt = 0;
            for (int t = -2; t <= 2; ++t) {
                int u = j + t;
                if (u < 0 || u >= SCAN_RADIAL_N) continue;
                acc += dev[u]; ++cnt;
            }
            sm[j] = acc / cnt;
        }

        // Nearest run of SCAN_MIN_RUN over threshold, but a marginal one is only
        // believed if nothing more convincing lies further out -- otherwise the
        // scan latches onto whatever weak artefact sits closest in.
        int best = -1, fallback = -1;
        int run = 0, start = -1;
        float peak = 0;
        for (int j = 0; j <= SCAN_RADIAL_N; ++j) {
            bool on = (j < SCAN_RADIAL_N) && mask_usable(i, j) && sm[j] > SCAN_THRESHOLD;
            if (on) {
                if (!run) { start = j; peak = 0; }
                ++run;
                if (sm[j] > peak) peak = sm[j];
            } else {
                if (run >= SCAN_MIN_RUN) {
                    if (fallback < 0) fallback = start;
                    if (peak >= SCAN_CONFIRM) { best = start; break; }
                }
                run = 0;
            }
        }
        if (best < 0) best = fallback;

        if (best < 0) {
            out->radius_px[i] = NAN;
            out->distance_cm[i] = NAN;
            out->strength[i] = 0;
        } else {
            float r = SCAN_R_IN + best * dr;
            out->radius_px[i] = r;
            float acc = 0; int cnt = 0;
            for (int t = 0; t < SCAN_MIN_RUN && best + t < SCAN_RADIAL_N; ++t) { acc += sm[best + t]; ++cnt; }
            out->strength[i] = cnt ? acc / cnt : 0;
            // the fit lives in calibration pixels, the scan in device pixels
            float r_cal = r * SCAN_CALIB_W / (float)SCAN_FRAME_W;
            float t = (r_cal - SCAN_FIT_A) / SCAN_FIT_B;
            out->distance_cm[i] = (t > 0 && t < (float)M_PI / 2 * 0.999f)
                                      ? SCAN_FIT_C * tanf(t) : NAN;
            ++out->hits;
        }
    }
    out->us_detect = (uint32_t)(esp_timer_get_time() - t0);
    return true;
}
