#include "connections.h"
#include "configurations.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

static uint16_t seqCounter = 0;
static volatile uint32_t sentOk = 0, sentFail = 0;
static volatile uint32_t failStreak = 0;
static volatile bool linkUp = true;              // optimistic until LINK_DOWN_FAILS sends in a row failed
static volatile bool linkUpEvent = true;         // first link-up counts as an event
static bool ready = false;
static uint16_t statusSeq = 0;

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
    if (status == ESP_NOW_SEND_SUCCESS) {
        sentOk++;
        failStreak = 0;
        if (!linkUp) { linkUp = true; linkUpEvent = true; }
    } else {
        sentFail++;
        if (++failStreak >= (uint32_t)LINK_DOWN_FAILS) linkUp = false;
    }
}

bool ESPNowInit() {
    logMsg(LOGLVL_INFO, "[NOW] initializing ESP-NOW");
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    WiFi.disconnect();
    WiFi.setSleep(false);                      // lowest latency
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    uint8_t mac[6];
    WiFi.macAddress(mac);
    logMsg(LOGLVL_INFO, "[NOW] master MAC %02X:%02X:%02X:%02X:%02X:%02X (put this in the slave if it filters senders)",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    if (esp_now_init() != ESP_OK) { logMsg(LOGLVL_ERROR, "[NOW] esp_now_init failed"); return false; }
    esp_now_register_send_cb(onSent);

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, SLAVE_MAC, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) { logMsg(LOGLVL_ERROR, "[NOW] add peer failed"); return false; }
    ready = true;
    logMsg(LOGLVL_INFO, "[NOW] ESP-NOW initialized, peer %02X:%02X:%02X:%02X:%02X:%02X%s", SLAVE_MAC[0], SLAVE_MAC[1], SLAVE_MAC[2],
           SLAVE_MAC[3], SLAVE_MAC[4], SLAVE_MAC[5], SLAVE_MAC[0] == 0xFF ? " (broadcast, no ACK)" : "");
    return true;
}

bool ESPNowSend(JointPacket &p) {
    p.start = PKT_START;
    p.type = PKT_TYPE_JOINTS;
    p.seq = seqCounter++;
    p.crc = crc8((const uint8_t *)&p, sizeof(p) - 1);
    return esp_now_send(SLAVE_MAC, (const uint8_t *)&p, sizeof(p)) == ESP_OK;
}

bool ESPNowReady() { return ready; }
bool ESPNowLinkUp() { return linkUp; }
bool ESPNowTakeLinkUpEvent() { const bool e = linkUpEvent; linkUpEvent = false; return e; }
bool ESPNowSendRaw(const uint8_t *data, size_t len) { return ready && esp_now_send(SLAVE_MAC, data, len) == ESP_OK; }

bool ESPNowSendStatus(StatusPacket &p) {
    p.start = PKT_START;
    p.type = PKT_TYPE_STATUS;
    p.seq = statusSeq++;
    p.sentOk = sentOk;
    p.sentFail = sentFail;
    p.logDropped = logDroppedCount();
    p.crc = crc8((const uint8_t *)&p, sizeof(p) - 1);
    return ESPNowSendRaw((const uint8_t *)&p, sizeof(p));
}

void ESPNowStats(uint32_t &ok, uint32_t &fail) { ok = sentOk; fail = sentFail; }
