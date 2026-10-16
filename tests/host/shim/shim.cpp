#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <vector>
#include <string>
static unsigned long g_micros = 0;
unsigned long micros() { return g_micros; }
unsigned long millis() { return g_micros / 1000; }
void shimSetMicros(unsigned long v) { g_micros = v; }
void (*shimDelayHook)(unsigned long) = 0;
void delay(unsigned long ms) { g_micros += ms * 1000UL; if (shimDelayHook) shimDelayHook(ms); }
SerialShim Serial; SPIClass SPI; TwoWire Wire; WiFiShim WiFi;

static int g_pin[64];
void shimPinMode(int, int) {}
void shimDigitalWrite(int p, int v) { if (p >= 0 && p < 64) g_pin[p] = v; }
int shimDigitalRead(int p) { return (p >= 0 && p < 64) ? g_pin[p] : 1; }
void shimSetPin(int p, int v) { if (p >= 0 && p < 64) g_pin[p] = v; }

// ESP-NOW loopback capture (tests read what the master "sent")
std::vector<std::vector<uint8_t> > g_espnowSent;
static esp_now_recv_cb_t g_recv = 0;
esp_err_t esp_now_init() { return ESP_OK; }
esp_err_t esp_now_register_send_cb(esp_now_send_cb_t) { return ESP_OK; }
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb) { g_recv = cb; return ESP_OK; }
esp_err_t esp_now_add_peer(const esp_now_peer_info_t *) { return ESP_OK; }
esp_err_t esp_now_send(const uint8_t *, const uint8_t *d, size_t n) { g_espnowSent.push_back(std::vector<uint8_t>(d, d + n)); return ESP_OK; }
void shimDeliver(const uint8_t *d, int n) { if (g_recv) { uint8_t mac[6] = { 1, 2, 3, 4, 5, 6 }; g_recv(mac, d, n); } }
