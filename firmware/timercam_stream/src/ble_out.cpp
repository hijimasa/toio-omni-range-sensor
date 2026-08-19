#include "ble_out.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <esp_camera.h>
#include <esp_timer.h>
#include <math.h>

#include "scan.h"

extern bool camera_set_mode(int mode);       // main.cpp
extern void logf(const char* fmt, ...);
extern float battery_volts();       // main.cpp

static NimBLECharacteristic* g_chr = nullptr;
static volatile bool g_connected = false;
static volatile uint32_t g_published = 0;
static volatile uint32_t g_dupes = 0;
// Minimum spacing between scans. The sensor will hand over ~11 frames a second
// and the scan itself costs ~52 ms of CPU, so running flat out keeps the core
// busy for most of the time. Nothing needs that: with a 12 cm range, 5 Hz still
// gives three looks at an obstacle for a robot doing 20 cm/s. Raise it if the
// robot moves faster; 0 means as fast as the sensor allows.
static volatile uint32_t g_scan_period_ms = 200;

#pragma pack(push, 1)
struct ScanPacket {
    uint16_t magic;
    uint8_t version;
    uint8_t n_rays;
    uint16_t seq;
    uint16_t vbat_mv;
    uint32_t uptime_s;
    uint16_t dist_mm[SCAN_N_RAYS];
};
#pragma pack(pop)

extern void power_off();            // main.cpp
extern volatile bool g_wifi_request;

class ScanCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo& info) override {
        std::string v = ch->getValue();
        if (v.empty()) return;
        switch ((uint8_t)v[0]) {
        case OMNI_CMD_REBOOT:
            logf("ble: reboot requested\n");
            delay(200);
            ESP.restart();
            break;
        case OMNI_CMD_POWER_OFF:
            logf("ble: power off requested\n");
            delay(200);
            power_off();
            break;
        case OMNI_CMD_SCAN_PERIOD:
            if (v.size() >= 3) {
                uint32_t ms = (uint8_t)v[1] | ((uint8_t)v[2] << 8);
                ble_set_scan_period(ms > 10000 ? 10000 : ms);
                logf("ble: scan period -> %u ms\n", ms);
            }
            break;
        case OMNI_CMD_WIFI_ON:
            // Not started here: bringing WiFi up from the BLE host task risks
            // its stack. loop() picks the request up.
            logf("ble: wifi requested\n");
            g_wifi_request = true;
            break;
        default:
            logf("ble: unknown command 0x%02x\n", (uint8_t)v[0]);
        }
    }
};

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
        g_connected = true;
        logf("ble: connected\n");
    }
    void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
        g_connected = false;
        logf("ble: disconnected (reason %d)\n", reason);
        NimBLEDevice::startAdvertising();   // else unreachable until reboot
    }
};

static void scan_task(void*) {
    static ScanResult res;
    static ScanPacket pkt;
    pkt.magic = 0x4F52;
    pkt.version = 2;
    pkt.n_rays = SCAN_N_RAYS;
    pkt.seq = 0;

    for (;;) {
        // Only measure while someone is listening. Idle advertising costs very
        // little; running the camera and the scan does not.
        if (!g_connected) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (!camera_set_mode(0 /* MODE_MEASURE */)) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        static uint32_t t_next = 0;
        uint32_t now = millis();
        if (g_scan_period_ms && (int32_t)(t_next - now) > 0) {
            vTaskDelay(pdMS_TO_TICKS(t_next - now));
            continue;
        }
        t_next = millis() + g_scan_period_ms;

        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        // With fb_count 2 the driver hands back whichever buffer is ready, and
        // that is often the one already scanned: measured, 32 % of scans were
        // re-processing a frame that had not changed. Skipping those saves the
        // CPU time and the BLE packet, and costs nothing -- the answer would
        // have been identical.
        static struct timeval last_ts = {0, 0};
        if (fb->timestamp.tv_sec == last_ts.tv_sec &&
            fb->timestamp.tv_usec == last_ts.tv_usec) {
            esp_camera_fb_return(fb);
            ++g_dupes;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        last_ts = fb->timestamp;

        // How old is the frame about to be measured? esp_camera stamps every
        // buffer at capture time, and with fb_count 2 the driver hands back the
        // oldest queued one -- so the answer is not zero, and every millisecond
        // of it is latency the operator sees in RViz. Measured rather than
        // assumed: the same "it must be fresh" assumption was wrong for /still
        // and quietly paired each calibration distance with the previous
        // target position.
        // esp32-camera stamps fb->timestamp from esp_timer_get_time(), so the
        // comparison has to use the same clock. Comparing against gettimeofday()
        // instead reads the drift between the RTC and the APB timer as if it
        // were latency -- it showed 730 ms, and "improved" to 555 ms purely
        // because a reflash had reset the uptime the drift accumulates over.
        int64_t fb_us = (int64_t)fb->timestamp.tv_sec * 1000000 + fb->timestamp.tv_usec;
        int32_t age_ms = (int32_t)((esp_timer_get_time() - fb_us) / 1000);

        bool ok = scan_frame(fb, &res);
        esp_camera_fb_return(fb);
        if (!ok) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        for (int i = 0; i < SCAN_N_RAYS; ++i) {
            float d = res.distance_cm[i];
            pkt.dist_mm[i] = (isnan(d) || d <= 0 || d > 6553.0f)
                                 ? OMNI_BLE_NO_ECHO
                                 : (uint16_t)lroundf(d * 10.0f);
        }
        ++pkt.seq;
        pkt.vbat_mv = (uint16_t)lroundf(battery_volts() * 1000.0f);
        pkt.uptime_s = millis() / 1000;
        g_chr->setValue((uint8_t*)&pkt, sizeof(pkt));
        g_chr->notify();
        ++g_published;
        // Latency budget, once every 2 s: how stale the frame was when it
        // arrived, and how long the whole publish cycle takes.
        {
            static uint32_t t_last_pub = 0, t_log = 0;
            static uint32_t age_sum = 0, cyc_sum = 0, n = 0;
            uint32_t now = millis();
            if (t_last_pub) { cyc_sum += now - t_last_pub; }
            t_last_pub = now;
            age_sum += (uint32_t)(age_ms < 0 ? 0 : age_ms);
            ++n;
            if (now - t_log > 2000 && n) {
                logf("latency: frame age %u ms, publish cycle %u ms, "
                     "scan %u ms (%u scans)\n",
                     (unsigned)(age_sum / n), (unsigned)(cyc_sum / (n > 1 ? n - 1 : 1)),
                     (unsigned)((res.us_sample + res.us_detect) / 1000), (unsigned)n);
                t_log = now; age_sum = cyc_sum = n = 0;
            }
        }

        // Short yield only: the duplicate check above is what paces the loop
        // to the sensor's actual frame rate now.
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void ble_start() {
    uint64_t mac = ESP.getEfuseMac();
    char name[32];
    snprintf(name, sizeof(name), "OmniRange-%04X", (uint16_t)(mac >> 32));

    logf("ble: init\n");
    NimBLEDevice::init(name);
    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    NimBLEService* svc = server->createService(OMNI_BLE_SERVICE_UUID);
    g_chr = svc->createCharacteristic(
        OMNI_BLE_SCAN_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE);
    g_chr->setCallbacks(new ScanCharCallbacks());

    static ScanPacket empty = {};
    empty.magic = 0x4F52;
    empty.version = 2;
    empty.n_rays = SCAN_N_RAYS;
    for (int i = 0; i < SCAN_N_RAYS; ++i) empty.dist_mm[i] = OMNI_BLE_NO_ECHO;
    g_chr->setValue((uint8_t*)&empty, sizeof(empty));

    svc->start();
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(OMNI_BLE_SERVICE_UUID);
    adv->enableScanResponse(true);
    NimBLEDevice::startAdvertising();

    xTaskCreatePinnedToCore(scan_task, "omni_scan", 8192, nullptr, 3, nullptr, 1);
    logf("ble: advertising as %s (%u byte packets)\n", name, (unsigned)sizeof(ScanPacket));
}

bool ble_connected() { return g_connected; }
uint32_t ble_scans_published() { return g_published; }
uint32_t ble_dupes_skipped() { return g_dupes; }
void ble_set_scan_period(uint32_t ms) { g_scan_period_ms = ms; }
uint32_t ble_scan_period() { return g_scan_period_ms; }
