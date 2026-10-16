#pragma once
#include <Arduino.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_NOW_ETH_ALEN 6
typedef enum { ESP_NOW_SEND_SUCCESS = 0, ESP_NOW_SEND_FAIL } esp_now_send_status_t;
typedef struct { uint8_t peer_addr[6]; uint8_t lmk[16]; uint8_t channel; int ifidx; bool encrypt; void *priv; } esp_now_peer_info_t;
typedef void (*esp_now_send_cb_t)(const uint8_t *mac, esp_now_send_status_t status);
typedef void (*esp_now_recv_cb_t)(const uint8_t *mac, const uint8_t *data, int len);
esp_err_t esp_now_init();
esp_err_t esp_now_register_send_cb(esp_now_send_cb_t cb);
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb);
esp_err_t esp_now_add_peer(const esp_now_peer_info_t *p);
esp_err_t esp_now_send(const uint8_t *mac, const uint8_t *data, size_t len);
