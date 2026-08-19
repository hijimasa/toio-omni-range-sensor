// On-device version of tools/omni_range.py.
//
// The point of doing this on the ESP32 is power: streaming JPEG costs ~26 kB a
// frame over WiFi, while the answer is 72 numbers. Everything here has to match
// the PC implementation, so the geometry, mask and range fit are generated from
// the same calibration files by tools/gen_firmware_calib.py.
#pragma once

#include <esp_camera.h>
#include <stdint.h>

#include "scan_calib.h"

struct ScanResult {
    float radius_px[SCAN_N_RAYS];   // device pixels; NaN where nothing was found
    float distance_cm[SCAN_N_RAYS]; // NaN past the modelled horizon
    float strength[SCAN_N_RAYS];
    uint8_t hits;
    // what the floor looked like this frame, useful when a scan comes back empty
    float floor_s, floor_s_mad, floor_v, floor_v_mad;
    uint32_t us_sample, us_detect;
};

// The camera mode the geometry was calibrated at, derived from the generated
// frame size so the two cannot drift apart. It has to be derived rather than
// chosen: several modes are a crop of the sensor rather than a scaled view of
// the same field, so a scan taken at the wrong one is not a rescaled scan, it
// is a different picture -- and scan_frame() would silently refuse every frame.
#if   SCAN_FRAME_W == 320 && SCAN_FRAME_H == 240
#define SCAN_FRAMESIZE FRAMESIZE_QVGA
#elif SCAN_FRAME_W == 320 && SCAN_FRAME_H == 320
#define SCAN_FRAMESIZE FRAMESIZE_320X320
#elif SCAN_FRAME_W == 400 && SCAN_FRAME_H == 296
#define SCAN_FRAMESIZE FRAMESIZE_CIF
#elif SCAN_FRAME_W == 480 && SCAN_FRAME_H == 320
#define SCAN_FRAMESIZE FRAMESIZE_HVGA
#elif SCAN_FRAME_W == 640 && SCAN_FRAME_H == 480
#define SCAN_FRAMESIZE FRAMESIZE_VGA
#else
#error "scan_calib.h asks for a frame size with no matching camera framesize"
#endif

// fb must be PIXFORMAT_RGB565 at SCAN_FRAME_W x SCAN_FRAME_H.
bool scan_frame(const camera_fb_t* fb, ScanResult* out);
