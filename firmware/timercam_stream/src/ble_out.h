// Publishes the 72 measured distances over BLE.
//
// This is what makes the power saving real: the measurement runs on the unit,
// so the only thing that has to leave it is 72 numbers. A scan is 148 bytes
// against ~26 kB for the JPEG that used to be streamed for the same result, and
// once nothing needs WiFi it can be switched off entirely (/radio?wifi=0).
//
// Payload, little-endian:
//   u16 magic 0x4F52 ("OR")
//   u8  version = 1
//   u8  n_rays  = 72
//   u16 seq                      scan counter, wraps
//   u16 vbat_mv                  battery, so a discharge run needs no USB
//   u32 uptime_s
//   u16 dist_mm[72]              0xFFFF = nothing found in that direction
// = 156 bytes. Battery and uptime ride along in the same packet rather than in
// a second characteristic: a discharge measurement has USB (and therefore
// serial) and WiFi off, so this is the only channel left, and one attribute is
// one thing that can go wrong. Bearing of ray i is i*5 deg in the ROS frame (x forward, CCW+),
// which is exactly the panorama column angle the calibration was fitted for.
#pragma once

#include <stdint.h>

#define OMNI_BLE_SERVICE_UUID "6f6d6e69-0001-4a52-b000-000000000001"
#define OMNI_BLE_SCAN_UUID    "6f6d6e69-0002-4a52-b000-000000000001"
#define OMNI_BLE_NO_ECHO      0xFFFF

// Starts advertising and the scan task. Safe to call once, after the camera.
void ble_start();

// Diagnostics for /status.
bool ble_connected();
uint32_t ble_scans_published();
uint32_t ble_dupes_skipped();   // frames handed back unchanged and skipped

// Commands, written to the scan characteristic (a write, not a second
// attribute: adding one earlier left it invisible behind a stale BlueZ GATT
// cache, and the scan characteristic is known to work).
//   0x01                 reboot
//   0x02                 power off (releases the LDO hold; battery only)
//   0x03 <u16 le>        set the scan period in ms
//   0x04                 start WiFi (for the calibration tools)
#define OMNI_CMD_REBOOT      0x01
#define OMNI_CMD_POWER_OFF   0x02
#define OMNI_CMD_SCAN_PERIOD 0x03
#define OMNI_CMD_WIFI_ON     0x04

// Scan pacing. Lower rate = less CPU time and fewer packets = longer battery.
void ble_set_scan_period(uint32_t ms);
uint32_t ble_scan_period();
