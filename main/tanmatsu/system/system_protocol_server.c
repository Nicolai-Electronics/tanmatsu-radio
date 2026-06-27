#include "system_protocol_server.h"
#include <dirent.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "esp_app_desc.h"
#include "esp_err.h"
#include "esp_hosted_peer_data.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "priv_events.h"

#define NVS_DEFAULT_PART_NAME "nvs"

static const char* TAG = "system";
static uint8_t     reply_buffer[512];

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

// NVS

static nvs_type_t system_protocol_nvs_value_type_to_idf(system_protocol_nvs_value_type_t type) {
    switch (type) {
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT8:
            return NVS_TYPE_U8;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT8:
            return NVS_TYPE_I8;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT16:
            return NVS_TYPE_U16;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT16:
            return NVS_TYPE_I16;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT32:
            return NVS_TYPE_U32;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT32:
            return NVS_TYPE_I32;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT64:
            return NVS_TYPE_U64;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT64:
            return NVS_TYPE_I64;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_STRING:
            return NVS_TYPE_STR;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_BLOB:
            return NVS_TYPE_BLOB;
        default:
            return NVS_TYPE_ANY;
    }
}

static system_protocol_nvs_value_type_t system_protocol_nvs_value_type_from_idf(nvs_type_t type) {
    switch (type) {
        case NVS_TYPE_U8:
            return SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT8;
            break;
        case NVS_TYPE_I8:
            return SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT8;
            break;
        case NVS_TYPE_U16:
            return SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT16;
            break;
        case NVS_TYPE_I16:
            return SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT16;
            break;
        case NVS_TYPE_U32:
            return SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT32;
            break;
        case NVS_TYPE_I32:
            return SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT32;
            break;
        case NVS_TYPE_U64:
            return SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT64;
            break;
        case NVS_TYPE_I64:
            return SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT64;
            break;
        case NVS_TYPE_STR:
            return SYSTEM_PROTOCOL_NVS_VALUE_TYPE_STRING;
            break;
        case NVS_TYPE_BLOB:
            return SYSTEM_PROTOCOL_NVS_VALUE_TYPE_BLOB;
            break;
        case NVS_TYPE_ANY:
        default:
            return SYSTEM_PROTOCOL_NVS_VALUE_TYPE_ANY;
    }
}

static void system_protocol_nvs_list(uint32_t sequence_number, const uint8_t* payload, size_t payload_length) {
    if (payload_length < sizeof(system_protocol_nvs_list_request_t)) {
        ESP_LOGW(TAG, "NVS list command received with insufficient data length: %zu bytes", payload_length);
        system_protocol_send_nack(sequence_number);
        return;
    }
    system_protocol_nvs_list_request_t* request                     = (system_protocol_nvs_list_request_t*)payload;
    request->namespace_name[SYSTEM_PROTOCOL_NVS_MAX_KEY_LENGTH - 1] = '\0';
    request->key[SYSTEM_PROTOCOL_NVS_MAX_KEY_LENGTH - 1]            = '\0';

    system_protocol_header_t* response_header = (system_protocol_header_t*)reply_buffer;
    response_header->sequence_number          = sequence_number;
    response_header->type                     = SYSTEM_PROTOCOL_TYPE_NVS_LIST;

    system_protocol_nvs_list_response_t* response_body =
        (system_protocol_nvs_list_response_t*)(&reply_buffer[sizeof(system_protocol_header_t)]);
    response_body->count = 0;

    system_protocol_nvs_entry_t* response_entries =
        (system_protocol_nvs_entry_t*)(&reply_buffer[sizeof(system_protocol_header_t) +
                                                     sizeof(system_protocol_nvs_list_response_t)]);

    uint32_t max_count =
        (sizeof(reply_buffer) - sizeof(system_protocol_header_t) - sizeof(system_protocol_nvs_list_response_t)) /
        sizeof(system_protocol_nvs_entry_t);

    nvs_iterator_t iterator;
    esp_err_t res = nvs_entry_find(NVS_DEFAULT_PART_NAME, request->namespace_name[0] ? request->namespace_name : NULL,
                                   system_protocol_nvs_value_type_to_idf(request->type), &iterator);
    while (res == ESP_OK && request->offset > 0) {
        res = nvs_entry_next(&iterator);
        request->offset--;
    }
    while (res == ESP_OK && response_body->count <= max_count) {
        nvs_entry_info_t info;
        nvs_entry_info(iterator, &info);
        if (strncmp(request->key, info.key, strlen(request->key)) != 0) {
            continue;  // Skip entry if key doesn't start with value provided as filter
        }
        snprintf(response_entries[response_body->count].key, SYSTEM_PROTOCOL_NVS_MAX_KEY_LENGTH, "%s", info.key);
        response_entries[response_body->count].type = system_protocol_nvs_value_type_from_idf(info.type);
        res                                         = nvs_entry_next(&iterator);
        response_body->count++;
    }
    nvs_release_iterator(iterator);

    generate_custom_event(TANMATSU_EVENT_SYSTEM, reply_buffer,
                          sizeof(system_protocol_header_t) + sizeof(system_protocol_nvs_list_response_t) +
                              sizeof(system_protocol_nvs_entry_t) * response_body->count);
}

static void system_protocol_nvs_read(uint32_t sequence_number, const uint8_t* payload, size_t payload_length) {
    if (payload_length < sizeof(system_protocol_nvs_location_t)) {
        ESP_LOGW(TAG, "NVS list command received with insufficient data length: %zu bytes", payload_length);
        system_protocol_send_nack(sequence_number);
        return;
    }

    system_protocol_nvs_location_t* request = (system_protocol_nvs_location_t*)payload;

    system_protocol_header_t* response_header = (system_protocol_header_t*)reply_buffer;
    response_header->sequence_number          = sequence_number;
    response_header->type                     = SYSTEM_PROTOCOL_TYPE_NVS_READ;

    system_protocol_nvs_value_t* response_body =
        (system_protocol_nvs_value_t*)(&reply_buffer[sizeof(system_protocol_header_t)]);
    memcpy(&response_body->location, request, sizeof(system_protocol_nvs_location_t));
    response_body->value_length = 0;

    nvs_handle_t nvs_handle;
    esp_err_t    res = nvs_open(request->namespace_name, NVS_READONLY, &nvs_handle);
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "NVS read command failed to open NVS namespace '%s'", request->namespace_name);
        system_protocol_send_nack(sequence_number);
        return;
    }

    switch (request->entry.type) {
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT8: {
            uint8_t value               = 0;
            res                         = nvs_get_u8(nvs_handle, request->entry.key, &value);
            response_body->value_u8     = value;
            response_body->value_length = sizeof(uint8_t);
            break;
        }
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT8: {
            int8_t value                = 0;
            res                         = nvs_get_i8(nvs_handle, request->entry.key, &value);
            response_body->value_i8     = value;
            response_body->value_length = sizeof(int8_t);
            break;
        }
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT16: {
            uint16_t value              = 0;
            res                         = nvs_get_u16(nvs_handle, request->entry.key, &value);
            response_body->value_u16    = value;
            response_body->value_length = sizeof(uint16_t);
            break;
        }
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT16: {
            int16_t value               = 0;
            res                         = nvs_get_i16(nvs_handle, request->entry.key, &value);
            response_body->value_i16    = value;
            response_body->value_length = sizeof(int16_t);
            break;
        }
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT32: {
            uint32_t value              = 0;
            res                         = nvs_get_u32(nvs_handle, request->entry.key, &value);
            response_body->value_u32    = value;
            response_body->value_length = sizeof(uint32_t);
            break;
        }
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT32: {
            int32_t value               = 0;
            res                         = nvs_get_i32(nvs_handle, request->entry.key, &value);
            response_body->value_i32    = value;
            response_body->value_length = sizeof(int32_t);
            break;
        }
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT64: {
            uint64_t value              = 0;
            res                         = nvs_get_u64(nvs_handle, request->entry.key, &value);
            response_body->value_u64    = value;
            response_body->value_length = sizeof(uint64_t);
            break;
        }
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT64: {
            int64_t value               = 0;
            res                         = nvs_get_i64(nvs_handle, request->entry.key, &value);
            response_body->value_i64    = value;
            response_body->value_length = sizeof(int64_t);
            break;
        }
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_STRING: {
            size_t required_length = 0;
            res                    = nvs_get_str(nvs_handle, request->entry.key, NULL, &required_length);
            if (res != ESP_OK) break;
            if (required_length >
                sizeof(reply_buffer) - sizeof(system_protocol_header_t) - sizeof(system_protocol_nvs_value_t)) {
                res = ESP_ERR_NO_MEM;
                break;
            }
            res = nvs_get_str(nvs_handle, request->entry.key, response_body->value_string, &required_length);
            response_body->value_length = required_length;
            break;
        }
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_BLOB: {
            size_t required_length = 0;
            res                    = nvs_get_blob(nvs_handle, request->entry.key, NULL, &required_length);
            if (res != ESP_OK) break;
            if (required_length >
                sizeof(reply_buffer) - sizeof(system_protocol_header_t) - sizeof(system_protocol_nvs_value_t)) {
                res = ESP_ERR_NO_MEM;
                break;
            }
            res = nvs_get_blob(nvs_handle, request->entry.key, response_body->value_blob, &required_length);
            response_body->value_length = required_length;
            break;
        }
        default:
            res = ESP_FAIL;
    }

    if (res != ESP_OK) {
        ESP_LOGW(TAG, "NVS read command failed to read key '%s' from NVS namespace '%s'", request->entry.key,
                 request->namespace_name);
        nvs_close(nvs_handle);
        system_protocol_send_nack(sequence_number);
        return;
    }

    nvs_close(nvs_handle);
    generate_custom_event(
        TANMATSU_EVENT_SYSTEM, reply_buffer,
        sizeof(system_protocol_header_t) + sizeof(system_protocol_nvs_value_t) + response_body->value_length);
}

static void system_protocol_nvs_write(uint32_t sequence_number, const uint8_t* payload, size_t payload_length) {
    if (payload_length < sizeof(system_protocol_nvs_value_t)) {
        ESP_LOGW(TAG, "NVS write command received with insufficient data length: %zu bytes", payload_length);
        system_protocol_send_nack(sequence_number);
        return;
    }

    system_protocol_nvs_value_t* request = (system_protocol_nvs_value_t*)payload;

    nvs_handle_t nvs_handle;
    esp_err_t    res = nvs_open(request->location.namespace_name, NVS_READWRITE, &nvs_handle);
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "NVS write command failed to open NVS namespace '%s'", request->location.namespace_name);
        system_protocol_send_nack(sequence_number);
        return;
    }

    switch (request->location.entry.type) {
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT8:
            res = nvs_set_u8(nvs_handle, request->location.entry.key, request->value_u8);
            break;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT8:
            res = nvs_set_i8(nvs_handle, request->location.entry.key, request->value_i8);
            break;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT16:
            res = nvs_set_u16(nvs_handle, request->location.entry.key, request->value_u16);
            break;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT16:
            res = nvs_set_i16(nvs_handle, request->location.entry.key, request->value_i16);
            break;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT32:
            res = nvs_set_u32(nvs_handle, request->location.entry.key, request->value_u32);
            break;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT32:
            res = nvs_set_i32(nvs_handle, request->location.entry.key, request->value_i32);
            break;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT64:
            res = nvs_set_u64(nvs_handle, request->location.entry.key, request->value_u64);
            break;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT64:
            res = nvs_set_i64(nvs_handle, request->location.entry.key, request->value_i64);
            break;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_STRING:
            if (payload_length < sizeof(system_protocol_nvs_value_t) + request->value_length) {
                res = ESP_ERR_INVALID_SIZE;
                break;
            }
            res = nvs_set_str(nvs_handle, request->location.entry.key, request->value_string);
            break;
        case SYSTEM_PROTOCOL_NVS_VALUE_TYPE_BLOB:
            if (payload_length < sizeof(system_protocol_nvs_value_t) + request->value_length) {
                res = ESP_ERR_INVALID_SIZE;
                break;
            }
            res = nvs_set_blob(nvs_handle, request->location.entry.key, request->value_blob, request->value_length);
            break;
        default:
            res = ESP_FAIL;
    }

    if (res != ESP_OK) {
        ESP_LOGW(TAG, "NVS write command failed to write key '%s' to NVS namespace '%s'", request->location.entry.key,
                 request->location.namespace_name);
        nvs_close(nvs_handle);
        system_protocol_send_nack(sequence_number);
        return;
    }

    res = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (res != ESP_OK) {
        ESP_LOGW(TAG, "NVS write command failed to commit NVS namespace '%s'", request->location.namespace_name);
        system_protocol_send_nack(sequence_number);
        return;
    }

    system_protocol_send_ack(sequence_number);
}

static void system_protocol_nvs_delete(uint32_t sequence_number, const uint8_t* payload, size_t payload_length) {
    if (payload_length < sizeof(system_protocol_nvs_location_t)) {
        ESP_LOGW(TAG, "NVS delete command received with insufficient data length: %zu bytes", payload_length);
        system_protocol_send_nack(sequence_number);
        return;
    }

    system_protocol_nvs_location_t* request = (system_protocol_nvs_location_t*)payload;

    nvs_handle_t nvs_handle;
    esp_err_t    res = nvs_open(request->namespace_name, NVS_READWRITE, &nvs_handle);
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "NVS delete command failed to open NVS namespace '%s'", request->namespace_name);
        system_protocol_send_nack(sequence_number);
        return;
    }

    nvs_type_t actual_type;
    res = nvs_find_key(nvs_handle, request->entry.key, &actual_type);
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "NVS delete command failed to find key '%s' in NVS namespace '%s'", request->entry.key,
                 request->namespace_name);
        nvs_close(nvs_handle);
        system_protocol_send_nack(sequence_number);
        return;
    }

    if (actual_type != system_protocol_nvs_value_type_to_idf(request->entry.type)) {
        ESP_LOGW(TAG, "NVS delete command type mismatch for key '%s' in NVS namespace '%s'", request->entry.key,
                 request->namespace_name);
        nvs_close(nvs_handle);
        system_protocol_send_nack(sequence_number);
        return;
    }

    res = nvs_erase_key(nvs_handle, request->entry.key);
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "NVS delete command failed to erase key '%s' from NVS namespace '%s'", request->entry.key,
                 request->namespace_name);
        nvs_close(nvs_handle);
        system_protocol_send_nack(sequence_number);
        return;
    }

    res = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (res != ESP_OK) {
        ESP_LOGW(TAG, "NVS delete command failed to commit NVS namespace '%s'", request->namespace_name);
        system_protocol_send_nack(sequence_number);
        return;
    }

    system_protocol_send_ack(sequence_number);
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
        case SYSTEM_PROTOCOL_TYPE_NVS_LIST:
            system_protocol_nvs_list(packet->sequence_number, payload, payload_length);
            break;
        case SYSTEM_PROTOCOL_TYPE_NVS_READ:
            system_protocol_nvs_read(packet->sequence_number, payload, payload_length);
            break;
        case SYSTEM_PROTOCOL_TYPE_NVS_WRITE:
            system_protocol_nvs_write(packet->sequence_number, payload, payload_length);
            break;
        case SYSTEM_PROTOCOL_TYPE_NVS_DELETE:
            system_protocol_nvs_delete(packet->sequence_number, payload, payload_length);
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
