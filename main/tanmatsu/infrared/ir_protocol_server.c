#include "ir_protocol_server.h"
#include <stdbool.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_hosted_peer_data.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "interface.h"
#include "ir_nec_encoder.h"
#include "nvs.h"
#include "priv_events.h"
#include "sdio_slave_api.h"
#include "tanmatsu_hardware.h"

static const char*          TAG         = "ir";
static rmt_channel_handle_t tx_channel  = NULL;
static rmt_encoder_handle_t nec_encoder = NULL;

extern uint8_t protocol_server_reply_buffer[512];

static void generate_custom_event(uint32_t event_id, uint8_t* event_data, size_t event_data_len) {
    esp_err_t res = esp_hosted_send_custom_data(event_id, event_data, event_data_len);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send event: %s", esp_err_to_name(res));
    }
}

static void infrared_protocol_send_nack(uint32_t sequence_number) {
    infrared_protocol_header_t packet = {
        .sequence_number = sequence_number,
        .type            = INFRARED_PROTOCOL_TYPE_NACK,
    };
    generate_custom_event(TANMATSU_EVENT_IR, (uint8_t*)&packet, sizeof(infrared_protocol_header_t));
}

static void infrared_protocol_send_ack(uint32_t sequence_number) {
    infrared_protocol_header_t packet = {
        .sequence_number = sequence_number,
        .type            = INFRARED_PROTOCOL_TYPE_ACK,
    };
    generate_custom_event(TANMATSU_EVENT_IR, (uint8_t*)&packet, sizeof(infrared_protocol_header_t));
}

static esp_err_t initialize_infrared_driver(void) {
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = 1000000,  // 1MHz, 1 tick = 1us
        .mem_block_symbols = 64,       // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't
                                 // queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num          = BSP_IR_TX,
    };

    esp_err_t res = rmt_new_tx_channel(&tx_channel_cfg, &tx_channel);
    if (res != ESP_OK) {
        return res;
    }

    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle   = 0.33,
        .frequency_hz = 38000,  // 38KHz
    };
    res = rmt_apply_carrier(tx_channel, &carrier_cfg);
    if (res != ESP_OK) {
        return res;
    }

    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = 1000000,  // 1MHz, 1 tick = 1us
    };

    res = rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder);
    if (res != ESP_OK) {
        return res;
    }

    return rmt_enable(tx_channel);
}

static void ir_protocol_get_information(uint32_t sequence_number) {
    infrared_protocol_header_t* packet = (infrared_protocol_header_t*)protocol_server_reply_buffer;
    packet->sequence_number            = sequence_number;
    packet->type                       = INFRARED_PROTOCOL_TYPE_GET_INFORMATION;
    infrared_protocol_information_t* information =
        (infrared_protocol_information_t*)(protocol_server_reply_buffer + sizeof(infrared_protocol_header_t));
    information->available = (tx_channel != NULL && nec_encoder != NULL);
    generate_custom_event(TANMATSU_EVENT_IR, protocol_server_reply_buffer,
                          sizeof(infrared_protocol_header_t) + sizeof(infrared_protocol_information_t));
}

static void ir_protocol_set_configuration(uint32_t sequence_number, uint8_t* payload, size_t payload_length) {
    if (payload_length != sizeof(infrared_protocol_configuration_t)) {
        ESP_LOGW(TAG, "Failed to apply infrared configuration: unexpected length (%u)", payload_length);
        infrared_protocol_send_nack(sequence_number);
        return;
    }

    infrared_protocol_configuration_t* config = (infrared_protocol_configuration_t*)payload;

    if (nec_encoder == NULL || tx_channel == NULL) {
        infrared_protocol_send_nack(sequence_number);
        return;
    }

    float duty_cycle = config->duty_cycle / 256.0f;

    if (config->frequency_hz < 2000) {
        config->frequency_hz = 2000;  // Minimum 2kHz
    }

    if (duty_cycle > 0.5f) {
        duty_cycle = 0.5f;  // Maximum 50%
    }

    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle   = duty_cycle,
        .frequency_hz = config->frequency_hz,
    };
    esp_err_t res = rmt_apply_carrier(tx_channel, &carrier_cfg);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to apply infrared configuration: %s", esp_err_to_name(res));
        infrared_protocol_send_nack(sequence_number);
        return;
    }

    infrared_protocol_send_ack(sequence_number);
}

static void ir_protocol_send_nec(uint32_t sequence_number, const uint8_t* data, size_t data_len) {
    if (data_len != sizeof(ir_nec_scan_code_t)) {
        ESP_LOGW(TAG, "Received IR data with unexpected length: %d", data_len);
        return;
    }

    if (tx_channel == NULL || nec_encoder == NULL) {
        ESP_LOGE(TAG, "Infrared not available");
        infrared_protocol_send_nack(sequence_number);
        return;
    }

    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,  // no loop
    };

    const ir_nec_scan_code_t* scan_code = (const ir_nec_scan_code_t*)data;
    esp_err_t res = rmt_transmit(tx_channel, nec_encoder, scan_code, sizeof(ir_nec_scan_code_t), &transmit_config);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send infrared NEC code");
        infrared_protocol_send_nack(sequence_number);
        return;
    }

    infrared_protocol_send_ack(sequence_number);
}

static void ir_protocol_packet_callback(uint32_t msg_id, const uint8_t* request_buffer, size_t request_length) {
    if (msg_id != TANMATSU_EVENT_IR) {
        ESP_LOGW(TAG, "Received message with unexpected ID: %d", msg_id);
        return;
    }
    if (request_length < sizeof(infrared_protocol_header_t)) {
        ESP_LOGW(TAG, "Received infrared protocol packet is too short: %zu bytes", request_length);
        infrared_protocol_send_nack(0);
        return;
    }

    infrared_protocol_header_t* packet = (infrared_protocol_header_t*)request_buffer;

    const uint8_t* payload        = request_buffer + sizeof(infrared_protocol_header_t);
    size_t         payload_length = request_length - sizeof(infrared_protocol_header_t);

    switch (packet->type) {
        case INFRARED_PROTOCOL_TYPE_ACK:
            // No-op, return ack
            infrared_protocol_send_ack(packet->sequence_number);
            break;
        case INFRARED_PROTOCOL_TYPE_NACK:
            // No-op, return nack
            infrared_protocol_send_nack(packet->sequence_number);
            break;
        case INFRARED_PROTOCOL_TYPE_GET_INFORMATION:
            ir_protocol_get_information(packet->sequence_number);
            break;
        case INFRARED_PROTOCOL_TYPE_SET_CONFIGURATION: {
            ir_protocol_set_configuration(packet->sequence_number, payload, payload_length);
        }
        case INFRARED_PROTOCOL_TYPE_SEND_NEC:
            ir_protocol_send_nec(packet->sequence_number, payload, payload_length);
            break;
        default:
            // Unknown type, return nack
            infrared_protocol_send_nack(packet->sequence_number);
            break;
    }
}

esp_err_t infrared_protocol_initialize(void) {
    esp_err_t res;

    res = infrared_protocol_reconfigure();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reconfigure: %s", esp_err_to_name(res));
        return res;
    }

    res = esp_hosted_register_custom_callback(TANMATSU_EVENT_IR, ir_protocol_packet_callback);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IR protocol callback: %s", esp_err_to_name(res));
        return res;
    }

    return ESP_OK;
}

esp_err_t infrared_protocol_reconfigure(void) {
    esp_err_t res = ESP_OK;
    if (tx_channel == NULL && nec_encoder == NULL) {
        uint8_t      revision = 0;
        nvs_handle_t nvs_handle;
        esp_err_t    res = nvs_open("system", NVS_READONLY, &nvs_handle);
        if (res == ESP_OK) {
            nvs_get_u8(nvs_handle, "board.rev", &revision);
            nvs_close(nvs_handle);
        }

        if (revision >= 2) {
            res = initialize_infrared_driver();
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initialize IR transmitter: %s", esp_err_to_name(res));
                return res;
            }
            ESP_LOGI(TAG, "Initialized the IR transmitter");
        } else {
            ESP_LOGI(TAG, "IR transmitter not available on this board revision");
        }
    }
    return res;
}
