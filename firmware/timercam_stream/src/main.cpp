// Timer Camera F -> MJPEG over WiFi.
//
// Step 1 of the toio omnidirectional range sensor: get a live image out of the
// unit so the convex mirror can be aimed and the 360 deg field of view checked.
//
//   http://<ip>/           overlay UI (mirror circle + 72 rays)
//   http://<ip>/still      single JPEG
//   http://<ip>/status     JSON of the current sensor settings
//   http://<ip>/ctrl?var=saturation&val=2
//   http://<ip>:81/stream  multipart MJPEG
//
// The stream lives on its own port because a long-lived multipart response
// would otherwise block every other request on the same server.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_http_server.h>
#include <esp_timer.h>
#include <img_converters.h>

#include "camera_pins.h"
#include "scan.h"
#include "ble_out.h"
#include "index_html.h"
#include "wifi_config.h"

static httpd_handle_t web_httpd    = nullptr;
static httpd_handle_t stream_httpd = nullptr;

// ---------------------------------------------------------------- logging ---
// Once the unit runs on battery the USB cable comes off -- and that cable is
// what shows up as a blind sector in the mirror -- so the serial console goes
// with it. Keep the last few KB of log in RAM and serve it over the existing
// HTTP server; no extra radio protocol, and nothing that competes with WiFi
// for airtime the way BLE would.

static const size_t LOG_CAP = 4096;
static char     g_log[LOG_CAP];
static size_t   g_log_head  = 0;     // next write position
static bool     g_log_wrapped = false;
static portMUX_TYPE g_log_mux = portMUX_INITIALIZER_UNLOCKED;

static void log_put(const char* s, size_t n) {
    portENTER_CRITICAL(&g_log_mux);
    for (size_t i = 0; i < n; ++i) {
        g_log[g_log_head] = s[i];
        if (++g_log_head >= LOG_CAP) {
            g_log_head = 0;
            g_log_wrapped = true;
        }
    }
    portEXIT_CRITICAL(&g_log_mux);
}

// Mirrors to the serial console, so behaviour is unchanged while USB is on.
void logf(const char* fmt, ...) {
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= sizeof(buf)) n = sizeof(buf) - 1;
    Serial.print(buf);
    log_put(buf, n);
}

// Minimum spacing between streamed frames. 2.4 GHz is usually crowded and the
// ESP32 soft AP loses the link long before it runs out of CPU, so the stream is
// throttled by default; raise the rate with /ctrl?var=interval&val=<ms>.
static volatile uint32_t g_frame_interval_ms = 150;

// Two camera modes. Measuring wants raw RGB565 at the scan resolution and no
// JPEG at all; the calibration tools want JPEG stills. Switching costs a camera
// re-init (~1 s including AEC settling), so the unit stays in whichever mode it
// was last asked for rather than flipping per request.
enum CamMode { MODE_MEASURE = 0, MODE_IMAGE = 1 };
bool g_wifi_disabled = false;
volatile bool g_wifi_request = false;
static volatile int g_mode = MODE_MEASURE;

static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=timercamframe";
static const char* STREAM_BOUNDARY     = "\r\n--timercamframe\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ---------------------------------------------------------------- camera ---

// Frame buffers per camera config. Kept as a name because camera_fb_get_fresh()
// has to drain exactly this many to be sure nothing in flight survives.
#define CAMERA_FB_COUNT 2

static bool camera_start(pixformat_t fmt = PIXFORMAT_JPEG,
                         framesize_t size = FRAMESIZE_UXGA,
                         int fb_count = -1) {
    camera_config_t cfg = {};
    cfg.pin_pwdn     = PWDN_GPIO_NUM;
    cfg.pin_reset    = RESET_GPIO_NUM;
    cfg.pin_xclk     = XCLK_GPIO_NUM;
    cfg.pin_sscb_sda = SIOD_GPIO_NUM;
    cfg.pin_sscb_scl = SIOC_GPIO_NUM;
    cfg.pin_d7       = Y9_GPIO_NUM;
    cfg.pin_d6       = Y8_GPIO_NUM;
    cfg.pin_d5       = Y7_GPIO_NUM;
    cfg.pin_d4       = Y6_GPIO_NUM;
    cfg.pin_d3       = Y5_GPIO_NUM;
    cfg.pin_d2       = Y4_GPIO_NUM;
    cfg.pin_d1       = Y3_GPIO_NUM;
    cfg.pin_d0       = Y2_GPIO_NUM;
    cfg.pin_vsync    = VSYNC_GPIO_NUM;
    cfg.pin_href     = HREF_GPIO_NUM;
    cfg.pin_pclk     = PCLK_GPIO_NUM;

    cfg.xclk_freq_hz = 20000000;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.pixel_format = fmt;
    // The frame buffers are sized here once and never grown, so /ctrl can only
    // ever select a framesize <= this one. Init at UXGA to keep the whole
    // usable range available, then drop to VGA below for a fast stream.
    cfg.frame_size   = size;
    cfg.jpeg_quality = 12;
    // RGB565 buffers are ~30x larger than a JPEG one (XGA = 1.5 MB each), so a
    // second one does not fit alongside anything else in 4 MB of PSRAM.
    cfg.fb_count     = fb_count > 0 ? fb_count
                                    : (psramFound() ? CAMERA_FB_COUNT : 1);
    cfg.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    // GRAB_LATEST keeps the driver capturing even with no client attached,
    // which floods the log with "cam_hal: FB-OVF". WHEN_EMPTY only captures
    // into a free buffer, and with fb_count 2 the stream stays just as fresh.
    cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        logf("camera init failed: 0x%x\n", err);
        return false;
    }

    sensor_t* s = esp_camera_sensor_get();
    if (s && fmt == PIXFORMAT_JPEG) {
        // VGA keeps the stream near 10 fps on the classic ESP32; the browser
        // UI and the Python tool can raise it once the mirror is aligned.
        // Only for the default boot config -- an explicit size must be kept.
        if (size == FRAMESIZE_UXGA) s->set_framesize(s, FRAMESIZE_VGA);
        // The mirror image is laterally reversed; leave the correction to the
        // unwrapping step so the raw stream stays predictable.
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
        // Saturation is the cue this sensor will eventually measure on, so
        // push it slightly and keep the gamma/lens-correction defaults.
        s->set_saturation(s, 1);
        // Auto white balance drifts with the illuminant and tints the whole
        // scene, which destroys the saturation contrast the range measurement
        // depends on. Do NOT "fix" that with set_whitebal(0) -- on the OV3660
        // that reverts to default gains and is far worse (measured: floor
        // saturation 21 -> 62). A fixed preset is the right lever; "sunny"
        // measured the most neutral floor of the five (contrast 10.3x vs 6.8x
        // for auto) under room lighting.
        s->set_wb_mode(s, 1);
    } else if (s) {
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
        s->set_saturation(s, 1);
        s->set_wb_mode(s, 1);
    }
    return true;
}

// A frame captured after this call, not before it.
//
// grab_mode is CAMERA_GRAB_WHEN_EMPTY (GRAB_LATEST floods the log with
// cam_hal FB-OVF when nobody is attached), and the driver refills a buffer the
// moment it is returned. So after a single-shot request hands its buffer back,
// the driver immediately captures into it -- and the next single-shot request
// gets THAT frame, taken at the time of the previous request. A stream never
// notices because frames keep flowing; /still and /scan do, and the picture
// they return is of whatever the world looked like one request ago. That is
// exactly the kind of wrong-but-plausible answer this sensor cannot afford:
// during a range calibration it silently pairs a known distance with the
// radius of the *previous* target position.
//
// Draining fb_count buffers discards everything already in flight, so the one
// taken afterwards can only have been captured after the drain started. It
// costs fb_count frame times (~180 ms at CIF), which is nothing for a
// single-shot endpoint.
static camera_fb_t* camera_fb_get_fresh() {
    for (int i = 0; i < CAMERA_FB_COUNT; ++i) {
        camera_fb_t* stale = esp_camera_fb_get();
        if (!stale) break;
        esp_camera_fb_return(stale);
    }
    return esp_camera_fb_get();
}

// How fast can the unit produce raw pixels? The eventual on-device version does
// not need a JPEG at all -- it samples ~72x200 points along the measurement
// rays -- so the question is the raw capture rate, not the encode rate.
static esp_err_t bench_handler(httpd_req_t* req) {
    char query[128], val[16];
    int size = FRAMESIZE_VGA, n = 8, rgb = 1, fb_req = 0, gray = 0;
    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen > 1 && qlen <= sizeof(query) &&
        httpd_req_get_url_query_str(req, query, qlen) == ESP_OK) {
        if (httpd_query_key_value(query, "size", val, sizeof(val)) == ESP_OK) size = atoi(val);
        if (httpd_query_key_value(query, "n", val, sizeof(val)) == ESP_OK) n = atoi(val);
        if (httpd_query_key_value(query, "rgb", val, sizeof(val)) == ESP_OK) rgb = atoi(val);
        if (httpd_query_key_value(query, "fb", val, sizeof(val)) == ESP_OK) fb_req = atoi(val);
        // grayscale is half the bytes of RGB565: if it is ~2x faster the
        // capture is memory-bandwidth bound, not sensor bound.
        if (httpd_query_key_value(query, "gray", val, sizeof(val)) == ESP_OK) gray = atoi(val);
    }
    n = constrain(n, 1, 30);

    bool decode = (rgb == 2);            // capture JPEG, then decode on-device
    pixformat_t fmt = gray ? PIXFORMAT_GRAYSCALE
                           : ((rgb == 1) ? PIXFORMAT_RGB565 : PIXFORMAT_JPEG);
    logf("bench: fmt=%s size=%d n=%d\n", rgb ? "RGB565" : "JPEG", size, n);

    esp_camera_deinit();
    delay(50);
    int fbc = fb_req > 0 ? fb_req : ((rgb == 1 || gray) ? 1 : 2);
    if (!camera_start(fmt, (framesize_t)size, fbc)) {
        // Leaving the camera down would brick every other endpoint, so put the
        // known-good JPEG config back before reporting the failure.
        esp_camera_deinit();
        delay(50);
        camera_start();
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"error\":\"init failed at this size/format\"}");
    }

    uint32_t heap_after = ESP.getFreeHeap();
    uint32_t psram_free = ESP.getFreePsram();
    for (int i = 0; i < 2; ++i) {          // discard the first frames
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
    }
    uint8_t* rgbbuf = nullptr;
    if (decode) {
        camera_fb_t* probe = esp_camera_fb_get();
        if (probe) {
            rgbbuf = (uint8_t*)ps_malloc((size_t)probe->width * probe->height * 2);
            esp_camera_fb_return(probe);
        }
        if (!rgbbuf) decode = false;     // no room: fall back to timing capture only
    }

    int64_t t0 = esp_timer_get_time();
    int64_t dt_dec = 0;
    size_t len = 0;
    int w = 0, h = 0, got = 0, dec_ok = 0;
    for (int i = 0; i < n; ++i) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) break;
        len = fb->len; w = fb->width; h = fb->height;
        if (decode) {
            int64_t d0 = esp_timer_get_time();
            if (jpg2rgb565(fb->buf, fb->len, rgbbuf, JPG_SCALE_NONE)) ++dec_ok;
            dt_dec += esp_timer_get_time() - d0;
        }
        esp_camera_fb_return(fb);
        ++got;
    }
    int64_t dt = esp_timer_get_time() - t0;
    if (rgbbuf) free(rgbbuf);

    esp_camera_deinit();
    delay(50);
    camera_start();                        // back to the JPEG streaming config

    char json[320];
    int m = snprintf(json, sizeof(json),
        "{\"format\":\"%s\",\"framesize\":%d,\"width\":%d,\"height\":%d,"
        "\"frames\":%d,\"bytes_per_frame\":%u,\"ms_per_frame\":%.1f,\"fps\":%.2f,"
        "\"decoded\":%d,\"ms_decode\":%.1f,\"fb_count\":%d,"
        "\"heap_after_init\":%u,\"psram_free_after_init\":%u}",
        gray ? "GRAYSCALE" : ((rgb == 1) ? "RGB565" : (decode ? "JPEG+decode" : "JPEG")),
        size, w, h, got, (unsigned)len,
        got ? dt / 1000.0 / got : -1.0, got ? got * 1e6 / dt : -1.0,
        dec_ok, dec_ok ? dt_dec / 1000.0 / dec_ok : -1.0, fbc,
        (unsigned)heap_after, (unsigned)psram_free);
    logf("bench: %dx%d %d frames %.1f ms/frame\n", w, h, got,
         got ? dt / 1000.0 / got : -1.0);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, m);
}

// esp_camera_init() wants internal DRAM that WiFi has already taken: with the
// station up the free internal heap is ~15 kB and the init fails at every size
// and format (measured: JPEG VGA with fb_count 1 fails just as UXGA does).
// Since the switch deinits first, attempting one in that state does not just
// fail -- it takes down the mode that was working, and the unit has no camera
// at all until it reboots. Refuse the switch instead. Boot has ~90 kB here.
static const uint32_t CAM_SWITCH_MIN_HEAP = 30000;

// One buffer in measure mode, not two. With two, the driver fills both and
// stops; the scan task then sleeps out its period and is handed the older of
// the pair. Measured on the unit: the frame being measured was 730 ms old, so
// every scan described where the obstacle had been three quarters of a second
// earlier. With one buffer there is nothing to queue -- esp_camera_fb_get()
// can only return the capture that just finished. Capture no longer overlaps
// the 48 ms of scanning, which costs throughput the scan period was throwing
// away anyway.
#define MEASURE_FB_COUNT 1

static bool camera_start_for(int mode) {
    return (mode == MODE_MEASURE)
               ? camera_start(PIXFORMAT_RGB565, (framesize_t)SCAN_FRAMESIZE,
                              MEASURE_FB_COUNT)
               : camera_start();
}

bool camera_set_mode(int mode) {
    if (mode == g_mode && esp_camera_sensor_get()) return true;
    uint32_t heap = ESP.getFreeHeap();
    if (heap < CAM_SWITCH_MIN_HEAP) {
        logf("camera mode %d refused: only %u B of heap (need %u). "
             "WiFi and the camera cannot both be brought up at runtime.\n",
             mode, (unsigned)heap, (unsigned)CAM_SWITCH_MIN_HEAP);
        return false;
    }
    int was = g_mode;
    esp_camera_deinit();
    delay(30);
    if (!camera_start_for(mode)) {
        // Put back what was running rather than the heavier JPEG config: the
        // point of the fallback is to leave the unit usable, and the mode we
        // came from is the one already known to fit.
        esp_camera_deinit(); delay(30);
        bool back = camera_start_for(was);
        g_mode = was;
        logf("camera mode %d failed, %s\n", mode,
             back ? "restored the previous mode" : "and the previous mode is gone too");
        return false;
    }
    g_mode = mode;
    // A re-init restarts AEC/AGC convergence; measuring a frame before it lands
    // shifts the floor brightness by tens of levels.
    for (int i = 0; i < 8; ++i) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
    }
    logf("camera mode -> %s\n", mode == MODE_MEASURE ? "measure (RGB565)" : "image (JPEG)");
    return true;
}

// ------------------------------------------------------------------ misc ---

// Releasing the LDO hold cuts the unit's own power. Only works on battery --
// with USB plugged in the rail stays up regardless. This exists because the
// reset button ends up under the mirror mount and is unreachable once built.
void power_off() {
    logf("powering off\n");
    Serial.flush();
    delay(50);
    digitalWrite(CAMERA_LED_GPIO, LOW);
    digitalWrite(POWER_HOLD_PIN, LOW);
    // If USB is supplying the rail this will not actually cut power, so make
    // the state obvious rather than pretending it worked.
    delay(1500);
    logf("still alive: USB is powering the rail, unplug it to switch off\n");
}

float battery_volts() {
    uint32_t mv = analogReadMilliVolts(BAT_ADC_PIN);
    return (mv / BAT_ADC_SCALE) / 1000.0f;
}

// --------------------------------------------------------------- handlers ---

static esp_err_t index_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t still_handler(httpd_req_t* req) {
    camera_set_mode(MODE_IMAGE);        // JPEG endpoints pull the camera back
    camera_fb_t* fb = camera_fb_get_fresh();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=timercam.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

static esp_err_t stream_handler(httpd_req_t* req) {
    camera_set_mode(MODE_IMAGE);
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char part[64];
    int64_t t_last = esp_timer_get_time();
    int64_t t_next = t_last;
    uint32_t n     = 0;

    while (true) {
        int64_t now = esp_timer_get_time();
        if (now < t_next) {
            vTaskDelay(pdMS_TO_TICKS((t_next - now) / 1000 + 1));
        }
        t_next = esp_timer_get_time() + (int64_t)g_frame_interval_ms * 1000;

        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            res = ESP_FAIL;
            break;
        }
        size_t len = fb->len;
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            int hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)len);
            res = httpd_resp_send_chunk(req, part, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char*)fb->buf, len);
        }
        esp_camera_fb_return(fb);
        if (res != ESP_OK) break;  // client disconnected

        if (++n % 30 == 0) {
            int64_t t_now = esp_timer_get_time();
            logf("stream %.1f fps, last frame %u B\n",
                          30.0 * 1000000.0 / (double)(t_now - t_last), (unsigned)len);
            t_last = t_now;
        }
    }
    return res;
}

// Returns true when the query string carried a "var"/"val" pair.
static bool get_ctrl_args(httpd_req_t* req, char* var, size_t var_len, int* val) {
    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen <= 1 || qlen > 128) return false;

    char query[128];
    if (httpd_req_get_url_query_str(req, query, qlen) != ESP_OK) return false;

    char sval[32];
    if (httpd_query_key_value(query, "var", var, var_len) != ESP_OK) return false;
    if (httpd_query_key_value(query, "val", sval, sizeof(sval)) != ESP_OK) return false;
    *val = atoi(sval);
    return true;
}

static esp_err_t ctrl_handler(httpd_req_t* req) {
    char var[32];
    int val = 0;
    if (!get_ctrl_args(req, var, sizeof(var), &val)) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    sensor_t* s = esp_camera_sensor_get();
    if (!s) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int res = 0;
    if (!strcmp(var, "framesize")) {
        camera_set_mode(MODE_IMAGE);
        s = esp_camera_sensor_get();
        if (s && s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
    }
    else if (!strcmp(var, "quality"))       res = s->set_quality(s, val);
    else if (!strcmp(var, "brightness"))    res = s->set_brightness(s, val);
    else if (!strcmp(var, "contrast"))      res = s->set_contrast(s, val);
    else if (!strcmp(var, "saturation"))    res = s->set_saturation(s, val);
    else if (!strcmp(var, "sharpness"))     res = s->set_sharpness(s, val);
    else if (!strcmp(var, "denoise"))       res = s->set_denoise(s, val);
    else if (!strcmp(var, "special_effect"))res = s->set_special_effect(s, val);
    else if (!strcmp(var, "awb"))           res = s->set_whitebal(s, val);
    else if (!strcmp(var, "awb_gain"))      res = s->set_awb_gain(s, val);
    else if (!strcmp(var, "wb_mode"))       res = s->set_wb_mode(s, val);
    else if (!strcmp(var, "aec"))           res = s->set_exposure_ctrl(s, val);
    else if (!strcmp(var, "aec2"))          res = s->set_aec2(s, val);
    else if (!strcmp(var, "ae_level"))      res = s->set_ae_level(s, val);
    else if (!strcmp(var, "aec_value"))     res = s->set_aec_value(s, val);
    else if (!strcmp(var, "agc"))           res = s->set_gain_ctrl(s, val);
    else if (!strcmp(var, "agc_gain"))      res = s->set_agc_gain(s, val);
    else if (!strcmp(var, "gainceiling"))   res = s->set_gainceiling(s, (gainceiling_t)val);
    else if (!strcmp(var, "lenc"))          res = s->set_lenc(s, val);
    else if (!strcmp(var, "raw_gma"))       res = s->set_raw_gma(s, val);
    else if (!strcmp(var, "hmirror"))       res = s->set_hmirror(s, val);
    else if (!strcmp(var, "vflip"))         res = s->set_vflip(s, val);
    else if (!strcmp(var, "led"))           digitalWrite(CAMERA_LED_GPIO, val ? HIGH : LOW);
    else if (!strcmp(var, "interval"))      g_frame_interval_ms = constrain(val, 0, 5000);
    else if (!strcmp(var, "scan_period"))  ble_set_scan_period(constrain(val, 0, 10000));
    else res = -1;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (res != 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t log_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Static, not a local: the httpd task stack is only a few KB and a
    // LOG_CAP-sized automatic array overflows it (observed as a hung request).
    // The web server handles one request at a time, so a shared buffer is safe.
    static char snap[LOG_CAP];
    size_t head;
    bool wrapped;
    portENTER_CRITICAL(&g_log_mux);
    memcpy(snap, g_log, LOG_CAP);
    head = g_log_head;
    wrapped = g_log_wrapped;
    portEXIT_CRITICAL(&g_log_mux);

    // Oldest-first: the tail of the buffer precedes the head once it has wrapped.
    if (wrapped) httpd_resp_send_chunk(req, snap + head, LOG_CAP - head);
    if (head) httpd_resp_send_chunk(req, snap, head);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t reboot_handler(httpd_req_t* req) {
    // Bulk throughput on this unit collapses after a few hours of streaming
    // (small requests and ping stay fine, so it is not the link dropping) and
    // only a restart clears it. On battery there is no USB to power-cycle from,
    // hence this.
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "rebooting\n");
    logf("reboot requested over HTTP\n");
    Serial.flush();
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP.restart();
    return ESP_OK;
}

// Run the whole measurement on the unit and return only the 72 numbers. This
// is the endpoint that makes the WiFi traffic ~170x smaller than streaming.
static esp_err_t scan_handler(httpd_req_t* req) {
    static ScanResult res;
    static char body[4096];

    if (!camera_set_mode(MODE_MEASURE)) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"error\":\"rgb565 mode unavailable\"}");
    }

    int64_t t0 = esp_timer_get_time();
    camera_fb_t* fb = camera_fb_get_fresh();
    int64_t t1 = esp_timer_get_time();
    bool ok = fb && scan_frame(fb, &res);
    if (fb) esp_camera_fb_return(fb);
    if (!ok) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"error\":\"scan failed\"}");
    }
    uint32_t us_capture = (uint32_t)(t1 - t0);

    int n = snprintf(body, sizeof(body),
        "{\"hits\":%u,\"floor\":{\"s\":%.1f,\"s_mad\":%.1f,\"v\":%.1f,\"v_mad\":%.1f},"
        "\"us\":{\"capture\":%u,\"sample\":%u,\"detect\":%u},\"scan\":[",
        res.hits, res.floor_s, res.floor_s_mad, res.floor_v, res.floor_v_mad,
        us_capture, res.us_sample, res.us_detect);
    for (int i = 0; i < SCAN_N_RAYS && n < (int)sizeof(body) - 96; ++i) {
        float bearing = i * (360.0f / SCAN_N_RAYS);
        if (bearing > 180.0f) bearing -= 360.0f;
        if (isnan(res.radius_px[i])) {
            n += snprintf(body + n, sizeof(body) - n, "%s[%.1f,null,null,0]", i ? "," : "", bearing);
        } else if (isnan(res.distance_cm[i])) {
            n += snprintf(body + n, sizeof(body) - n, "%s[%.1f,%.1f,null,%.1f]",
                          i ? "," : "", bearing, res.radius_px[i], res.strength[i]);
        } else {
            n += snprintf(body + n, sizeof(body) - n, "%s[%.1f,%.1f,%.2f,%.1f]",
                          i ? "," : "", bearing, res.radius_px[i],
                          res.distance_cm[i], res.strength[i]);
        }
    }
    n += snprintf(body + n, sizeof(body) - n, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, body, n);
}

// Turning WiFi off is the whole point of the BLE path: with the measurement
// running on the unit there is nothing left that needs 126 kB/s. HTTP goes away
// with it, so the only way back is a reboot (power button, or the watchdog).
static esp_err_t radio_handler(httpd_req_t* req) {
    char query[64], val[16];
    bool off = false;
    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen > 1 && qlen <= sizeof(query) &&
        httpd_req_get_url_query_str(req, query, qlen) == ESP_OK &&
        httpd_query_key_value(query, "wifi", val, sizeof(val)) == ESP_OK) {
        off = (atoi(val) == 0);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, off ? "{\"wifi\":\"stopping\"}" : "{\"wifi\":\"on\"}");
    if (off) {
        logf("wifi off by request; BLE only from here, reboot to get HTTP back\n");
        delay(300);
        g_wifi_disabled = true;
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }
    return ESP_OK;
}

static esp_err_t power_handler(httpd_req_t* req) {
    char query[64], val[16];
    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    bool reboot = false, off = false;
    if (qlen > 1 && qlen <= sizeof(query) &&
        httpd_req_get_url_query_str(req, query, qlen) == ESP_OK) {
        if (httpd_query_key_value(query, "off", val, sizeof(val)) == ESP_OK) off = atoi(val) != 0;
        if (httpd_query_key_value(query, "reboot", val, sizeof(val)) == ESP_OK) reboot = atoi(val) != 0;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, off ? "{\"power\":\"off\"}"
                               : (reboot ? "{\"power\":\"reboot\"}" : "{\"power\":\"on\"}"));
    delay(200);
    if (reboot) ESP.restart();
    if (off) power_off();
    return ESP_OK;
}

static esp_err_t mode_handler(httpd_req_t* req) {
    char query[64], val[16];
    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen > 1 && qlen <= sizeof(query) &&
        httpd_req_get_url_query_str(req, query, qlen) == ESP_OK &&
        httpd_query_key_value(query, "m", val, sizeof(val)) == ESP_OK) {
        camera_set_mode(strcmp(val, "image") == 0 ? MODE_IMAGE : MODE_MEASURE);
    }
    char json[96];
    int n = snprintf(json, sizeof(json), "{\"mode\":\"%s\"}",
                     g_mode == MODE_MEASURE ? "measure" : "image");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, n);
}

static esp_err_t status_handler(httpd_req_t* req) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    camera_status_t* c = &s->status;

    char json[512];
    int n = snprintf(json, sizeof(json),
        "{\"framesize\":%u,\"quality\":%u,\"brightness\":%d,\"contrast\":%d,"
        "\"saturation\":%d,\"sharpness\":%d,\"special_effect\":%u,\"wb_mode\":%u,"
        "\"awb\":%u,\"awb_gain\":%u,\"aec\":%u,\"aec2\":%u,\"ae_level\":%d,"
        "\"aec_value\":%u,\"agc\":%u,\"agc_gain\":%u,\"gainceiling\":%u,"
        "\"lenc\":%u,\"raw_gma\":%u,\"hmirror\":%u,\"vflip\":%u,"
        "\"interval\":%u,\"scan_period\":%u,\"ble\":%d,\"ble_scans\":%u,\"vbat\":%.2f,\"heap\":%u,\"rssi\":%d}",
        c->framesize, c->quality, c->brightness, c->contrast,
        c->saturation, c->sharpness, c->special_effect, c->wb_mode,
        c->awb, c->awb_gain, c->aec, c->aec2, c->ae_level,
        c->aec_value, c->agc, c->agc_gain, c->gainceiling,
        c->lenc, c->raw_gma, c->hmirror, c->vflip,
        (unsigned)g_frame_interval_ms, (unsigned)ble_scan_period(),
        ble_connected() ? 1 : 0, (unsigned)ble_scans_published(),
        battery_volts(), (unsigned)ESP.getFreeHeap(), (int)WiFi.RSSI());

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, n);
}

// Must not be called before WiFi has been brought up: httpd creates sockets,
// and without the network stack initialised that asserts inside FreeRTOS
// (xQueueSemaphoreTake on a NULL queue) and boot-loops the unit.
static void server_start() {
    if (web_httpd) return;                   // already running
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 14;
    config.stack_size = 8192;   // default 4096 is tight for these handlers
    // A client that dies mid-stream leaves its socket allocated; without this
    // the next stream request is refused until the old one times out.
    config.lru_purge_enable = true;

    httpd_uri_t index_uri  = {"/",       HTTP_GET, index_handler,  NULL};
    httpd_uri_t still_uri  = {"/still",  HTTP_GET, still_handler,  NULL};
    httpd_uri_t ctrl_uri   = {"/ctrl",   HTTP_GET, ctrl_handler,   NULL};
    httpd_uri_t status_uri = {"/status", HTTP_GET, status_handler, NULL};
    httpd_uri_t log_uri    = {"/log",    HTTP_GET, log_handler,    NULL};
    httpd_uri_t reboot_uri = {"/reboot", HTTP_GET, reboot_handler, NULL};
    httpd_uri_t bench_uri  = {"/bench",  HTTP_GET, bench_handler,  NULL};
    httpd_uri_t scan_uri   = {"/scan",   HTTP_GET, scan_handler,   NULL};
    httpd_uri_t mode_uri   = {"/mode",   HTTP_GET, mode_handler,   NULL};
    httpd_uri_t radio_uri  = {"/radio",  HTTP_GET, radio_handler,  NULL};
    httpd_uri_t power_uri  = {"/power",  HTTP_GET, power_handler,  NULL};
    httpd_uri_t stream_uri = {"/stream", HTTP_GET, stream_handler, NULL};

    if (httpd_start(&web_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(web_httpd, &index_uri);
        httpd_register_uri_handler(web_httpd, &still_uri);
        httpd_register_uri_handler(web_httpd, &ctrl_uri);
        httpd_register_uri_handler(web_httpd, &status_uri);
        httpd_register_uri_handler(web_httpd, &log_uri);
        httpd_register_uri_handler(web_httpd, &reboot_uri);
        httpd_register_uri_handler(web_httpd, &bench_uri);
        httpd_register_uri_handler(web_httpd, &scan_uri);
        httpd_register_uri_handler(web_httpd, &mode_uri);
        httpd_register_uri_handler(web_httpd, &radio_uri);
        httpd_register_uri_handler(web_httpd, &power_uri);
    }

    config.server_port += 1;  // 81
    config.ctrl_port += 1;    // separate control socket for the second server
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}

// ------------------------------------------------------------------ setup ---

static void wifi_start() {
    WiFi.persistent(false);
    // Modem sleep adds latency and drops packets under a sustained stream.
    WiFi.setSleep(false);

    if (strlen(WIFI_SSID) > 0) {
        WiFi.mode(WIFI_STA);
#ifdef USE_STATIC_IP
        WiFi.config(IPAddress(STATIC_IP), IPAddress(STATIC_GATEWAY),
                    IPAddress(STATIC_SUBNET), IPAddress(STATIC_DNS));
#endif
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        // The progress dots go to serial only -- they would just be noise in
        // the ring buffer that /log serves.
        logf("joining \"%s\"\n", WIFI_SSID);
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
            delay(250);
            Serial.print('.');
        }
        Serial.println();
        if (WiFi.status() == WL_CONNECTED) {
            logf("STA  http://%s/\n", WiFi.localIP().toString().c_str());
            logf("     http://%s:81/stream\n", WiFi.localIP().toString().c_str());
            return;
        }
        logf("join failed, falling back to soft AP\n");
    }

    WiFi.mode(WIFI_AP);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    char ssid[40];
    uint64_t mac = ESP.getEfuseMac();
    snprintf(ssid, sizeof(ssid), "%s-%04X", AP_SSID, (uint16_t)(mac >> 32));
    // max_connection 1: extra associated clients only steal airtime here.
    WiFi.softAP(ssid, strlen(AP_PASS) ? AP_PASS : NULL, AP_CHANNEL, 0, 1);
    logf("AP   \"%s\"  ch%d  http://%s/\n", ssid, AP_CHANNEL,
                  WiFi.softAPIP().toString().c_str());
    logf("     http://%s:81/stream\n", WiFi.softAPIP().toString().c_str());
}

void setup() {
    // Do this first: on battery the unit cuts its own power without it.
    pinMode(POWER_HOLD_PIN, OUTPUT);
    digitalWrite(POWER_HOLD_PIN, HIGH);
    pinMode(CAMERA_LED_GPIO, OUTPUT);
    digitalWrite(CAMERA_LED_GPIO, LOW);

    Serial.begin(115200);
    delay(100);
    logf("\nTimer Camera F omni stream\n");
    logf("PSRAM: %s (%u B)\n", psramFound() ? "yes" : "no", (unsigned)ESP.getPsramSize());

    // Arduino 3.x only accepts an attenuation change once the pin has been
    // attached to an ADC channel, so take one throwaway reading first.
    analogRead(BAT_ADC_PIN);
    analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);

    // Whichever configuration this build needs has to be started here, before
    // any radio, and never switched afterwards -- see camera_set_mode(). With
    // WiFi at boot this is the calibration build, so come up in the JPEG
    // configuration /still and /stream need; otherwise it is the measurement
    // build and the camera stays in RGB565 for scan_frame().
#if START_WIFI_AT_BOOT
    g_mode = MODE_IMAGE;
#else
    g_mode = MODE_MEASURE;
#endif
    if (!camera_start_for(g_mode)) {
        Serial.println("halted");
        while (true) {
            digitalWrite(CAMERA_LED_GPIO, !digitalRead(CAMERA_LED_GPIO));
            delay(200);
        }
    }

    // BLE first, and by default the only radio: WiFi is the expensive one and
    // nothing needs it once the measurement runs on the unit. The HTTP server
    // is started either way; it simply has no interface to answer on until
    // WiFi comes up (BLE command 0x04, or START_WIFI_AT_BOOT).
#if START_WIFI_AT_BOOT
    wifi_start();
    server_start();
    // and no BLE. The camera, the WiFi station and NimBLE do not fit in
    // internal RAM together -- any two do, all three do not. With the JPEG
    // camera and the station up, npl_freertos_callout_init() aborts with
    // ESP_ERR_NO_MEM and the unit bootloops. This build exists to serve
    // /still and /stream while the mirror is being calibrated; nothing in
    // that job needs BLE.
    logf("calibration build: WiFi + JPEG camera from boot, BLE off\n");
#else
    g_wifi_disabled = true;
    logf("wifi off at boot (BLE only); send the wifi-on command to enable it\n");
    ble_start();
#endif
    logf("battery %.2f V\n", battery_volts());
    // Leave the LED off. The mirror reflects it straight back into the annulus,
    // where it reads as a strongly saturated blue object: with it lit the scan
    // reported 3 phantom obstacles on an empty floor (strength up to 7.9),
    // with it off, 0/72. /ctrl?var=led&val=1 still turns it on for debugging.
    digitalWrite(CAMERA_LED_GPIO, LOW);
}

void loop() {
    // The unit has been seen to keep running while its WiFi stays down, which
    // looks exactly like "the camera died" from the outside. Reboot rather than
    // sit there unreachable -- STA mode re-joins on its own in a few seconds.
    static uint32_t t_lost = 0;
    if (!g_wifi_disabled && WiFi.getMode() == WIFI_STA) {
        if (WiFi.status() != WL_CONNECTED) {
            if (t_lost == 0) {
                t_lost = millis();
                logf("wifi lost\n");
            } else if (millis() - t_lost > 20000) {
                logf("wifi down 20s, restarting\n");
                Serial.flush();
                ESP.restart();
            }
        } else if (t_lost != 0) {
            logf("wifi back after %u ms\n", (unsigned)(millis() - t_lost));
            t_lost = 0;
        }
    }

    if (g_wifi_request) {
        g_wifi_request = false;
        g_wifi_disabled = false;
        logf("starting wifi on request\n");
        wifi_start();
        server_start();
    }

    // Heartbeat into the ring buffer. On battery this is the only way to see
    // how the pack is holding up and how long the unit has actually been up.
    static uint32_t t_beat = 0;
    if (millis() - t_beat >= 60000) {
        t_beat = millis();
        logf("up %6us  vbat %.2f V  heap %6u  rssi %d\n",
             (unsigned)(millis() / 1000), battery_volts(),
             (unsigned)ESP.getFreeHeap(), (int)WiFi.RSSI());
    }
    delay(1000);
}
