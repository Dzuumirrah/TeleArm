#pragma once
#include <esp_now.h>
#define WIFI_SECOND_CHAN_NONE 0
#define WIFI_IF_STA 0
inline esp_err_t esp_wifi_set_channel(uint8_t, int) { return ESP_OK; }
