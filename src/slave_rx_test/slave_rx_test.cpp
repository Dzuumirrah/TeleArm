// Receiver test for the SLAVE ESP32: checks the master's JointPacket and prints it.
// The real slave firmware only has to do the same validation and then drive the servos
// with packet.joint_deg[0..2] (a, b, d) - no kinematics are needed there.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "configurations.h"
#include "connections.h"

static volatile bool     g_new = false;
static JointPacket       g_pkt;
static volatile uint32_t g_ok = 0, g_bad = 0, g_lost = 0;
static uint16_t          g_lastSeq = 0;
static bool              g_haveSeq = false;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t *, const uint8_t *data, int len) {
#else
static void onRecv(const uint8_t *, const uint8_t *data, int len) {
#endif
    if (!packetValid(data, len)) { g_bad++; return; }
    JointPacket p;
    memcpy(&p, data, sizeof(p));
    if (g_haveSeq) g_lost += (uint16_t)(p.seq - g_lastSeq - 1);
    g_lastSeq = p.seq; g_haveSeq = true;
    g_pkt = p; g_new = true; g_ok++;
}

void setup() {
    Serial.begin(115200);
    delay(300);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.print("slave MAC: "); Serial.println(WiFi.macAddress());   // put it in SLAVE_MAC (configurations.cpp)
    if (esp_now_init() != ESP_OK) { Serial.println("esp_now_init failed"); return; }
    esp_now_register_recv_cb(onRecv);
}

void loop() {
    static uint32_t lastPrint = 0;
    if (g_new && millis() - lastPrint >= 100) {
        g_new = false; lastPrint = millis();
        JointPacket p = g_pkt;
        Serial.printf("seq=%u a=%.1f b=%.1f d=%.1f flags=0x%02X ok=%lu bad=%lu lost=%lu\n", p.seq,
                      p.joint_deg[0], p.joint_deg[1], p.joint_deg[2], p.flags,
                      (unsigned long)g_ok, (unsigned long)g_bad, (unsigned long)g_lost);
    }
    delay(1);
}
