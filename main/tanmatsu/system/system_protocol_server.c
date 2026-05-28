#include "system_protocol_server.h"
#include <stdio.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_err.h"
#include "esp_hosted_peer_data.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "interface.h"
#include "priv_events.h"
#include "sdio_slave_api.h"
#include "tanmatsu_hardware.h"

static const char* TAG = "system";

static void generate_custom_event(uint32_t event_id, uint8_t* event_data, size_t event_data_len) {
    esp_err_t res = esp_hosted_send_custom_data(event_id, event_data, event_data_len);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send lora event: %s", esp_err_to_name(res));
    }
}

static void system_protocol_send_nack(uint32_t sequence_number) {
    system_protocol_header_t packet = {
        .sequence_number = sequence_number,
        .type            = SYSTEM_PROTOCOL_TYPE_NACK,
    };
    generate_custom_event(TANMATSU_EVENT_SYSTEM, (uint8_t*)&packet, sizeof(system_protocol_header_t));
}

static void system_protocol_send_ack(uint32_t sequence_number) {
    system_protocol_header_t packet = {
        .sequence_number = sequence_number,
        .type            = SYSTEM_PROTOCOL_TYPE_ACK,
    };
    generate_custom_event(TANMATSU_EVENT_SYSTEM, (uint8_t*)&packet, sizeof(system_protocol_header_t));
}

static void system_protocol_get_information(uint32_t sequence_number) {
    const esp_app_desc_t*     app_description = esp_app_get_description();
    uint8_t                   buffer[sizeof(system_protocol_header_t) + sizeof(system_protocol_information_t)];
    system_protocol_header_t* packet = (system_protocol_header_t*)buffer;
    packet->sequence_number          = sequence_number;
    packet->type                     = SYSTEM_PROTOCOL_TYPE_GET_INFORMATION;
    system_protocol_information_t* information =
        (system_protocol_information_t*)(buffer + sizeof(system_protocol_header_t));
    memcpy(information->firmware_name, app_description->project_name, 32);
    memcpy(information->firmware_version, app_description->version, 32);
    memcpy(information->firmware_build_date, app_description->date, 16);
    memcpy(information->firmware_build_time, app_description->time, 16);
    memcpy(information->firmware_idf_version, app_description->idf_ver, 32);
    memcpy(information->firmware_sha256, app_description->app_elf_sha256, 32);
    generate_custom_event(TANMATSU_EVENT_SYSTEM, buffer, sizeof(buffer));
}

static void system_protocol_packet_callback(uint32_t msg_id, const uint8_t* request_buffer, size_t request_length) {
    if (msg_id != TANMATSU_EVENT_SYSTEM) {
        ESP_LOGW(TAG, "Received message with unexpected ID: %d", msg_id);
        return;
    }
    if (request_length < sizeof(system_protocol_header_t)) {
        ESP_LOGW(TAG, "Received system protocol packet is too short: %zu bytes", request_length);
        system_protocol_send_nack(0);
        return;
    }

    system_protocol_header_t* packet = (system_protocol_header_t*)request_buffer;

    const uint8_t* payload        = request_buffer + sizeof(system_protocol_header_t);
    size_t         payload_length = request_length - sizeof(system_protocol_header_t);

    switch (packet->type) {
        case SYSTEM_PROTOCOL_TYPE_ACK:
            // No-op, return ack
            system_protocol_send_ack(packet->sequence_number);
            break;
        case SYSTEM_PROTOCOL_TYPE_NACK:
            // No-op, return nack
            system_protocol_send_nack(packet->sequence_number);
            break;
        case SYSTEM_PROTOCOL_TYPE_GET_INFORMATION:
            system_protocol_get_information(packet->sequence_number);
            break;
        default:
            // Unknown type, return nack
            system_protocol_send_nack(packet->sequence_number);
            break;
    }
}

esp_err_t system_protocol_initialize(void) {
    esp_err_t res = esp_hosted_register_custom_callback(TANMATSU_EVENT_SYSTEM, system_protocol_packet_callback);
    if (res != ESP_OK) {
        return res;
    }
    return ESP_OK;
}
