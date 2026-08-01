#include "echo_protocol_server.h"
#include <stdio.h>
#include "esp_err.h"
#include "esp_hosted_peer_data.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "interface.h"
#include "priv_events.h"
#include "sdio_slave_api.h"
#include "hardware.h"

static const char* TAG = "echo";

static void echo_protocol_packet_callback(uint32_t msg_id, const uint8_t* data, size_t data_len) {
    if (msg_id != TANMATSU_EVENT_ECHO) {
        ESP_LOGW(TAG, "Received message with unexpected ID: %d", msg_id);
        return;
    }
    esp_err_t res = esp_hosted_send_custom_data(msg_id, data, data_len);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send echo event: %s", esp_err_to_name(res));
    }
}

esp_err_t echo_protocol_initialize(void) {
    esp_err_t res = esp_hosted_register_custom_callback(TANMATSU_EVENT_ECHO, echo_protocol_packet_callback);
    if (res != ESP_OK) {
        return res;
    }
    return ESP_OK;
}
