// SPDX-FileCopyrightText: 2026 Nicolai Electronics
// SPDX-License-Identifier: MIT

#include "lora_protocol_server.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_hosted_peer_data.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "interface.h"
#include "lora.h"
#include "priv_events.h"
#include "sdio_slave_api.h"
#include "tanmatsu_hardware.h"

#define LORA_PACKET_QUEUE_SIZE 32

static const char* TAG = "lora";

static lora_handle_t lora_handle = {0};
static uint8_t       reply_buffer[512];

static void generate_custom_event(uint32_t event_id, uint8_t* event_data, size_t event_data_len) {
    esp_err_t res = esp_hosted_send_custom_data(event_id, event_data, event_data_len);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send lora event: %s", esp_err_to_name(res));
    }
}

static void lora_protocol_send_nack(uint32_t sequence_number) {
    lora_protocol_header_t* nack_packet = (lora_protocol_header_t*)reply_buffer;
    nack_packet->sequence_number        = sequence_number;
    nack_packet->type                   = LORA_PROTOCOL_TYPE_NACK;
    size_t nack_length                  = sizeof(lora_protocol_header_t);
    generate_custom_event(TANMATSU_EVENT_LORA, reply_buffer, nack_length);
}

static void lora_protocol_send_ack(uint32_t sequence_number) {
    lora_protocol_header_t* ack_packet = (lora_protocol_header_t*)reply_buffer;
    ack_packet->sequence_number        = sequence_number;
    ack_packet->type                   = LORA_PROTOCOL_TYPE_ACK;
    size_t ack_length                  = sizeof(lora_protocol_header_t);
    generate_custom_event(TANMATSU_EVENT_LORA, reply_buffer, ack_length);
}

static void lora_protocol_get_mode(uint32_t sequence_number) {
    lora_protocol_header_t* mode_packet = (lora_protocol_header_t*)reply_buffer;
    mode_packet->sequence_number        = sequence_number;
    mode_packet->type                   = LORA_PROTOCOL_TYPE_GET_MODE;
    lora_protocol_mode_params_t* mode_params =
        (lora_protocol_mode_params_t*)(reply_buffer + sizeof(lora_protocol_header_t));
    lora_protocol_mode_t mode = LORA_PROTOCOL_MODE_UNKNOWN;
    esp_err_t            res  = lora_get_mode(&lora_handle, &mode);
    if (res == ESP_OK) {
        mode_params->mode    = mode;
        size_t status_length = sizeof(lora_protocol_header_t) + sizeof(lora_protocol_mode_params_t);
        generate_custom_event(TANMATSU_EVENT_LORA, reply_buffer, status_length);
    } else {
        lora_protocol_send_nack(sequence_number);
    }
}

static void lora_protocol_set_mode(uint32_t sequence_number, const uint8_t* payload, size_t payload_length) {
    if (payload_length < sizeof(lora_protocol_mode_params_t)) {
        ESP_LOGW(TAG, "Set mode command received with insufficient data length: %zu bytes", payload_length);
        lora_protocol_send_nack(sequence_number);
        return;
    }
    lora_protocol_mode_params_t* mode_params = (lora_protocol_mode_params_t*)payload;
    esp_err_t                    res         = lora_set_mode(&lora_handle, mode_params->mode);
    if (res == ESP_OK) {
        lora_protocol_send_ack(sequence_number);
    } else {
        lora_protocol_send_nack(sequence_number);
    }
}

static void lora_protocol_get_config(uint32_t sequence_number) {
    lora_protocol_header_t* config_packet = (lora_protocol_header_t*)reply_buffer;
    config_packet->sequence_number        = sequence_number;
    config_packet->type                   = LORA_PROTOCOL_TYPE_GET_CONFIG;
    lora_protocol_config_params_t* config_params =
        (lora_protocol_config_params_t*)(reply_buffer + sizeof(lora_protocol_header_t));
    esp_err_t res = lora_get_config(&lora_handle, config_params);
    if (res == ESP_OK) {
        size_t status_length = sizeof(lora_protocol_header_t) + sizeof(lora_protocol_config_params_t);
        generate_custom_event(TANMATSU_EVENT_LORA, reply_buffer, status_length);
    } else {
        lora_protocol_send_nack(sequence_number);
    }
}

static void lora_protocol_set_config(uint32_t sequence_number, const uint8_t* config_data, size_t config_length) {
    if (config_length < sizeof(lora_protocol_config_params_t)) {
        ESP_LOGW(TAG, "Set config command received with insufficient data length: %zu bytes", config_length);
        lora_protocol_send_nack(sequence_number);
        return;
    }
    lora_protocol_config_params_t* config_params = (lora_protocol_config_params_t*)config_data;
    printf("LoRa configuration: frequency=%" PRIu32
           " sf=%u bw=%u cr=%u sync=0x%02x preamble=%u power=%u ramp=%u crc=%d invert_iq=%d ldr_opt=%d rx_boost=%d\n",
           config_params->frequency, config_params->spreading_factor, config_params->bandwidth,
           config_params->coding_rate, config_params->sync_word, config_params->preamble_length, config_params->power,
           config_params->ramp_time, config_params->crc_enabled, config_params->invert_iq,
           config_params->low_data_rate_optimization, config_params->rx_boost);
    esp_err_t res = lora_set_config(&lora_handle, config_params);
    if (res == ESP_OK) {
        lora_protocol_send_ack(sequence_number);
    } else {
        lora_protocol_send_nack(sequence_number);
    }
}

static void lora_protocol_get_status(uint32_t sequence_number) {
    lora_protocol_header_t* status_packet = (lora_protocol_header_t*)reply_buffer;
    status_packet->sequence_number        = sequence_number;
    status_packet->type                   = LORA_PROTOCOL_TYPE_GET_STATUS;
    lora_protocol_status_params_t* status_params =
        (lora_protocol_status_params_t*)(reply_buffer + sizeof(lora_protocol_header_t));
    esp_err_t res = lora_get_status(&lora_handle, status_params);
    if (res == ESP_OK) {
        size_t status_length = sizeof(lora_protocol_header_t) + sizeof(lora_protocol_status_params_t);
        generate_custom_event(TANMATSU_EVENT_LORA, reply_buffer, status_length);
    } else {
        lora_protocol_send_nack(sequence_number);
    }
}

static void lora_protocol_packet_tx(uint32_t sequence_number, const uint8_t* packet_data, size_t packet_length) {
    lora_protocol_lora_packet_t packet = {0};
    if (packet_length > 256) {
        lora_protocol_send_nack(sequence_number);
    }
    memcpy(packet.data, packet_data, packet_length);
    packet.length = packet_length;
    esp_err_t res = lora_send_packet(&lora_handle, &packet);
    if (res == ESP_OK) {
        lora_protocol_send_ack(sequence_number);
    } else {
        lora_protocol_send_nack(sequence_number);
    }
}

static void lora_protocol_get_rssi_inst(uint32_t sequence_number) {
    float     signal_power = 0;
    esp_err_t res          = lora_get_rssi_inst(&lora_handle, &signal_power);
    if (res == ESP_OK) {
        int raw_int = (int)(-2.0f * signal_power + 0.5f);
        if (raw_int < 0) raw_int = 0;
        if (raw_int > 255) raw_int = 255;
        lora_protocol_header_t* rssi_inst_packet = (lora_protocol_header_t*)reply_buffer;
        rssi_inst_packet->sequence_number        = sequence_number;
        rssi_inst_packet->type                   = LORA_PROTOCOL_TYPE_GET_RSSI_INST;
        lora_protocol_rssi_inst_params_t* params =
            (lora_protocol_rssi_inst_params_t*)(reply_buffer + sizeof(lora_protocol_header_t));
        params->rssi_raw    = (uint8_t)raw_int;
        size_t reply_length = sizeof(lora_protocol_header_t) + sizeof(lora_protocol_rssi_inst_params_t);
        generate_custom_event(TANMATSU_EVENT_LORA, reply_buffer, reply_length);
    } else {
        lora_protocol_send_nack(sequence_number);
    }
}

static void lora_protocol_packet_callback(uint32_t msg_id, const uint8_t* request_buffer, size_t request_length) {
    if (msg_id != TANMATSU_EVENT_LORA) {
        ESP_LOGW(TAG, "Received message with unexpected ID: %d", msg_id);
        return;
    }

    if (request_length < sizeof(lora_protocol_header_t)) {
        ESP_LOGW(TAG, "Received LoRa protocol packet is too short: %zu bytes", request_length);
        lora_protocol_send_nack(0);
        return;
    }

    lora_protocol_header_t* packet = (lora_protocol_header_t*)request_buffer;

    const uint8_t* payload        = request_buffer + sizeof(lora_protocol_header_t);
    size_t         payload_length = request_length - sizeof(lora_protocol_header_t);

    switch (packet->type) {
        case LORA_PROTOCOL_TYPE_ACK:
            // No-op, return ack
            lora_protocol_send_ack(packet->sequence_number);
            break;
        case LORA_PROTOCOL_TYPE_NACK:
            // No-op, return nack
            lora_protocol_send_nack(packet->sequence_number);
            break;
        case LORA_PROTOCOL_TYPE_GET_MODE:
            lora_protocol_get_mode(packet->sequence_number);
            break;
        case LORA_PROTOCOL_TYPE_SET_MODE:
            lora_protocol_set_mode(packet->sequence_number, payload, payload_length);
            break;
        case LORA_PROTOCOL_TYPE_GET_CONFIG:
            lora_protocol_get_config(packet->sequence_number);
            break;
        case LORA_PROTOCOL_TYPE_SET_CONFIG:
            lora_protocol_set_config(packet->sequence_number, payload, payload_length);
            break;
        case LORA_PROTOCOL_TYPE_GET_STATUS:
            lora_protocol_get_status(packet->sequence_number);
            break;
        case LORA_PROTOCOL_TYPE_PACKET_RX:
            // We don't expect to receive this from the host, return nack
            lora_protocol_send_nack(packet->sequence_number);
            break;
        case LORA_PROTOCOL_TYPE_PACKET_TX:
            lora_protocol_packet_tx(packet->sequence_number, payload, payload_length);
            break;
        case LORA_PROTOCOL_TYPE_GET_RSSI_INST:
            lora_protocol_get_rssi_inst(packet->sequence_number);
            break;
        default:
            // Unknown type, return nack
            lora_protocol_send_nack(packet->sequence_number);
            break;
    }
}

static void lora_packet_receive_task(void* pvParameters) {
    while (1) {
        uint8_t                 data[sizeof(lora_protocol_header_t) + sizeof(lora_protocol_lora_packet_t) + 255] = {0};
        lora_protocol_header_t* header = (lora_protocol_header_t*)data;
        header->sequence_number        = 0;  // Sequence number is not used
        header->type                   = LORA_PROTOCOL_TYPE_PACKET_RX;
        lora_protocol_lora_packet_t* lora_packet =
            (lora_protocol_lora_packet_t*)(data + sizeof(lora_protocol_header_t));
        if (lora_receive_packet(&lora_handle, lora_packet, portMAX_DELAY) == ESP_OK) {
            size_t total_length =
                sizeof(lora_protocol_header_t) + sizeof(lora_packet_stats_t) + sizeof(uint8_t) + lora_packet->length;
            generate_custom_event(TANMATSU_EVENT_LORA, data, total_length);
        }
    }
    vTaskDelete(NULL);
}

esp_err_t lora_protocol_initialize(void) {
    esp_err_t res = lora_init_local(&lora_handle, LORA_PACKET_QUEUE_SIZE, BSP_LORA_BUS, BSP_LORA_CS, BSP_LORA_RESET,
                                    BSP_LORA_DIO1, BSP_LORA_BUSY);
    if (res != ESP_OK) {
        return res;
    }

    xTaskCreatePinnedToCore(lora_packet_receive_task, "lora_packet_receive_task", 4096, NULL, 10, NULL,
                            CONFIG_SOC_CPU_CORES_NUM - 1);

    return esp_hosted_register_custom_callback(TANMATSU_EVENT_LORA, lora_protocol_packet_callback);
}
