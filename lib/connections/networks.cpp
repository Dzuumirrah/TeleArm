#include "connections.h"
#include "configurations.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

static uint16_t seqCounter = 0;
static volatile uint32_t sentOk = 0, sentFail = 0;

uint8_t crc8(const uint8_t *d, size_t n) {
    uint8_t c = 0;
    while (n--) {
        c ^= *d++;
        for (int i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

bool packetValid(const uint8_t *d, int len) {
    if (len != (int)sizeof(JointPacket)) return false;
    if (d[0] != PKT_START || d[1] != PKT_TYPE_JOINTS) return false;
    return crc8(d, len - 1) == d[len - 1];
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onSent(const wifi_tx_info_t *, esp_now_send_status_t status) {
#else
static void onSent(const uint8_t *, esp_now_send_status_t status) {
#endif
    if (status == ESP_NOW_SEND_SUCCESS) sentOk++; else sentFail++;
}

bool ESPNowInit() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false);                      // lowest latency
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("[NOW] master MAC %02X:%02X:%02X:%02X:%02X:%02X (put this in the slave if it filters senders)\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    if (esp_now_init() != ESP_OK) { Serial.println("[NOW] esp_now_init failed"); return false; }
    esp_now_register_send_cb(onSent);

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, SLAVE_MAC, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) { Serial.println("[NOW] add peer failed"); return false; }
    return true;
}

bool ESPNowSend(JointPacket &p) {
    p.start = PKT_START;
    p.type = PKT_TYPE_JOINTS;
    p.seq = seqCounter++;
    p.crc = crc8((const uint8_t *)&p, sizeof(p) - 1);
    return esp_now_send(SLAVE_MAC, (const uint8_t *)&p, sizeof(p)) == ESP_OK;
}

void ESPNowStats(uint32_t &ok, uint32_t &fail) { ok = sentOk; fail = sentFail; }
