// SPDX-License-Identifier: MIT

#include "airplane_mode.h"
#include <string.h>
#include "esp_hosted_peer_data.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "priv_events.h"
#include "slave_bt.h"

static const char* TAG = "airplane_mode";

#define NVS_NAMESPACE "airplane"
#define NVS_KEY       "enabled"

// Protocol command bytes
#define AIRPLANE_CMD_GET      0x00
#define AIRPLANE_CMD_SET      0x01
#define AIRPLANE_RESP         0x02

static volatile bool s_airplane_enabled = false;

static void apply_airplane_mode(bool enable) {
    if (enable) {
        ESP_LOGI(TAG, "Activating airplane mode: stopping WiFi and Bluetooth");
        esp_err_t res = esp_wifi_stop();
        if (res != ESP_OK && res != ESP_ERR_WIFI_NOT_INIT) {
            ESP_LOGW(TAG, "esp_wifi_stop() returned: %s", esp_err_to_name(res));
        }
        res = disable_bluetooth();
        if (res != ESP_OK) {
            ESP_LOGW(TAG, "disable_bluetooth() returned: %s", esp_err_to_name(res));
        }
    } else {
        ESP_LOGI(TAG, "Deactivating airplane mode: radios may be re-enabled by host");
    }
}

static esp_err_t save_to_nvs(bool enabled) {
    nvs_handle_t handle;
    esp_err_t    res = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(res));
        return res;
    }
    res = nvs_set_u8(handle, NVS_KEY, enabled ? 1 : 0);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write NVS: %s", esp_err_to_name(res));
        nvs_close(handle);
        return res;
    }
    res = nvs_commit(handle);
    nvs_close(handle);
    return res;
}

static bool load_from_nvs(void) {
    nvs_handle_t handle;
    esp_err_t    res = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (res != ESP_OK) {
        return false;  // Default: airplane mode off
    }
    uint8_t value = 0;
    res           = nvs_get_u8(handle, NVS_KEY, &value);
    nvs_close(handle);
    if (res != ESP_OK) {
        return false;  // Default: airplane mode off
    }
    return value != 0;
}

static void send_response(bool enabled) {
    uint8_t response[2] = {AIRPLANE_RESP, enabled ? 1 : 0};
    esp_err_t res = esp_hosted_send_custom_data(TANMATSU_EVENT_AIRPLANE, response, sizeof(response));
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send airplane mode response: %s", esp_err_to_name(res));
    }
}

static void airplane_mode_packet_callback(uint32_t msg_id, const uint8_t* data, size_t data_len) {
    if (msg_id != TANMATSU_EVENT_AIRPLANE) {
        ESP_LOGW(TAG, "Received message with unexpected ID: %" PRIu32, msg_id);
        return;
    }

    if (data == NULL || data_len < 1) {
        ESP_LOGW(TAG, "Invalid airplane mode packet");
        return;
    }

    uint8_t command = data[0];

    switch (command) {
        case AIRPLANE_CMD_GET:
            send_response(s_airplane_enabled);
            break;
        case AIRPLANE_CMD_SET:
            if (data_len < 2) {
                ESP_LOGW(TAG, "SET command missing value byte");
                return;
            }
            bool new_state = data[1] != 0;
            if (new_state != s_airplane_enabled) {
                save_to_nvs(new_state);
                s_airplane_enabled = new_state;
                apply_airplane_mode(new_state);
            }
            send_response(s_airplane_enabled);
            break;
        default:
            ESP_LOGW(TAG, "Unknown airplane mode command: 0x%02x", command);
            break;
    }
}

esp_err_t airplane_mode_init(void) {
    s_airplane_enabled = load_from_nvs();
    ESP_LOGI(TAG, "Airplane mode initialized: %s", s_airplane_enabled ? "ENABLED" : "disabled");
    if (s_airplane_enabled) {
        apply_airplane_mode(true);
    }

    esp_err_t res = esp_hosted_register_custom_callback(TANMATSU_EVENT_AIRPLANE, airplane_mode_packet_callback);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register airplane mode callback: %s", esp_err_to_name(res));
        return res;
    }

    return ESP_OK;
}

bool airplane_mode_is_enabled(void) {
    return s_airplane_enabled;
}
