#pragma once
#include <Arduino.h>
#define WIFI_STA 1
struct WiFiShim {
    bool mode(int) { return true; }
    void disconnect() {}
    void macAddress(uint8_t *m) { for (int i = 0; i < 6; i++) m[i] = i; }
    const char *macAddress() { return "00:01:02:03:04:05"; }
    void setSleep(bool) {}
};
extern WiFiShim WiFi;
