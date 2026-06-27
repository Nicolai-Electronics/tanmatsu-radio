#include "badgelink_protocol_server.h"
#include <stdio.h>
#include "esp_err.h"
#include "esp_hosted_peer_data.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "interface.h"
#include "priv_events.h"
#include "sdio_slave_api.h"
#include "tanmatsu_hardware.h"
#include "badgelink.h"

static const char* TAG = "badgelink";

static void badgelink_tx_callback(uint8_t const* data, size_t len) {
    esp_err_t res = esp_hosted_send_custom_data(TANMATSU_EVENT_BADGELINK, data, len);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Badgelink event: %s", esp_err_to_name(res));
    }
}

static void badgelink_protocol_packet_callback(uint32_t msg_id, const uint8_t* data, size_t data_len) {
    if (msg_id != TANMATSU_EVENT_BADGELINK) {
        ESP_LOGW(TAG, "Received Badgelink message with unexpected ID: %d", msg_id);
        return;
    }
    badgelink_rxdata_cb(data, data_len);
}

esp_err_t badgelink_protocol_initialize(void) {
    badgelink_init();
    esp_err_t res = esp_hosted_register_custom_callback(TANMATSU_EVENT_BADGELINK, badgelink_protocol_packet_callback);
    if (res != ESP_OK) {
        return res;
    }
    badgelink_start(badgelink_tx_callback);
    return ESP_OK;
}
