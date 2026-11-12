// Receiver test for the SLAVE ESP32. The laptop is connected to THIS board, so it also shows the master's monitor:
//   JOINT  frame -> validated, printed at 10 Hz   "seq=.. a=.. b=.. d=.. flags=.. ok=.. bad=.. lost=.."
//   LOG    frame -> every line the master printed, in order:   "[M 12.345 W] [CAL] not still - ..."   (" R" = replayed)
//   STATUS frame -> 1 Hz heartbeat:                              "[M STATUS 13.001] state=RUN plan=- upper=MPU9250 ..."
// Lines produced by the slave itself start with "[S]". The real slave firmware only has to do the same validation
// and then drive the servos with packet.joint_deg[0..2] (a, b, d) - no kinematics are needed there.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "configurations.h"
#include "connections.h"

// The receive callback runs in the WiFi task: it only copies the frame into a ring, loop() does all printing.
static const int RX_SLOTS = 24;
struct RxSlot { uint8_t len; uint8_t data[LOG_HEADER_LEN + LOG_MAX_TEXT + 1]; };
static RxSlot            g_rx[RX_SLOTS];
static volatile uint32_t g_rxWr = 0, g_rxRd = 0, g_rxOverrun = 0;

static JointPacket       g_pkt;                      // newest joint frame (printed at 10 Hz)
static volatile bool     g_new = false;
static volatile uint32_t g_ok = 0, g_bad = 0, g_lost = 0;
static uint16_t          g_lastSeq = 0;
static bool              g_haveSeq = false;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t *, const uint8_t *data, int len) {
#else
static void onRecv(const uint8_t *, const uint8_t *data, int len) {
#endif
    const uint8_t type = packetType(data, len);
    if (type == 0) { g_bad++; return; }
    if (type == PKT_TYPE_JOINTS) {
        JointPacket p;
        memcpy(&p, data, sizeof(p));
        if (g_haveSeq) g_lost += (uint16_t)(p.seq - g_lastSeq - 1);
        g_lastSeq = p.seq; g_haveSeq = true;
        g_pkt = p; g_new = true; g_ok++;
        return;
    }
    if (g_rxWr - g_rxRd >= (uint32_t)RX_SLOTS) { g_rxOverrun++; return; }      // loop() is behind: drop the newest
    RxSlot &s = g_rx[g_rxWr % RX_SLOTS];
    s.len = (uint8_t)len;
    memcpy(s.data, data, (size_t)len);
    g_rxWr++;
}

static void printMonitorFrame(const RxSlot &s) {
    static bool haveLogSeq = false;
    static uint16_t lastLogSeq = 0;
    char line[300];
    const uint8_t type = packetType(s.data, s.len);
    if (type == PKT_TYPE_LOG) {
        LogFrame f;
        if (!decodeLogPacket(s.data, s.len, f)) return;
        if (haveLogSeq && (uint16_t)(f.seq - lastLogSeq - 1) != 0)
            Serial.printf("[S] %u master log line(s) lost in the air\n", (unsigned)(uint16_t)(f.seq - lastLogSeq - 1));
        lastLogSeq = f.seq; haveLogSeq = true;
        formatLogLine(f, line, sizeof(line));
        Serial.println(line);
    } else if (type == PKT_TYPE_STATUS) {
        StatusPacket st;
        if (!decodeStatusPacket(s.data, s.len, st)) return;
        formatStatusLine(st, line, sizeof(line));
        Serial.println(line);
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.print("[S] slave MAC: "); Serial.println(WiFi.macAddress());   // put it in SLAVE_MAC (configurations.cpp)
    if (esp_now_init() != ESP_OK) { Serial.println("[S] esp_now_init failed"); return; }
    esp_now_register_recv_cb(onRecv);
    Serial.println("[S] waiting for the master (its log and status are forwarded here)");
}

void loop() {
    while (g_rxRd != g_rxWr) {                           // master log / status first, in order
        printMonitorFrame(g_rx[g_rxRd % RX_SLOTS]);
        g_rxRd++;
    }
    static uint32_t lastPrint = 0, lastOverrun = 0;
    if (g_rxOverrun != lastOverrun) { Serial.printf("[S] %lu monitor frame(s) dropped (slave too slow)\n", (unsigned long)(g_rxOverrun - lastOverrun)); lastOverrun = g_rxOverrun; }
    if (g_new && millis() - lastPrint >= 100) {
        g_new = false; lastPrint = millis();
        JointPacket p = g_pkt;
        Serial.printf("seq=%u a=%.1f b=%.1f d=%.1f flags=0x%02X ok=%lu bad=%lu lost=%lu\n", p.seq,
                      p.joint_deg[0], p.joint_deg[1], p.joint_deg[2], p.flags,
                      (unsigned long)g_ok, (unsigned long)g_bad, (unsigned long)g_lost);
    }
    delay(1);
}
