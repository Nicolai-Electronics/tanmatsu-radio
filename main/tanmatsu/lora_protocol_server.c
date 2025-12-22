#include "lora_protocol_server.h"
#include <stddef.h>
#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "interface.h"
#include "lora_protocol.h"
#include "priv_events.h"
#include "sdio_slave_api.h"
#include "sx126x.h"
#include "tanmatsu_hardware.h"
#include "tanmatsu_interfaces.h"

static const char*                   TAG         = "lora";
static sx126x_handle_t               lora_handle = {0};
static uint8_t                       reply_buffer[512];
static lora_protocol_config_params_t current_config = {0};

static void send_nack(uint32_t sequence_number) {
    lora_protocol_header_t* nack_packet = (lora_protocol_header_t*)reply_buffer;
    nack_packet->sequence_number        = sequence_number;
    nack_packet->type                   = LORA_PROTOCOL_TYPE_NACK;
    size_t nack_length                  = sizeof(lora_protocol_header_t);
    generate_custom_event(ESP_PRIV_EVENT_LORA, reply_buffer, nack_length);
}

static void send_ack(uint32_t sequence_number) {
    lora_protocol_header_t* ack_packet = (lora_protocol_header_t*)reply_buffer;
    ack_packet->sequence_number        = sequence_number;
    ack_packet->type                   = LORA_PROTOCOL_TYPE_ACK;
    size_t ack_length                  = sizeof(lora_protocol_header_t);
    generate_custom_event(ESP_PRIV_EVENT_LORA, reply_buffer, ack_length);
}

static void send_mode(uint32_t sequence_number) {
    lora_protocol_header_t* mode_packet = (lora_protocol_header_t*)reply_buffer;
    mode_packet->sequence_number        = sequence_number;
    mode_packet->type                   = LORA_PROTOCOL_TYPE_GET_MODE;
    lora_protocol_mode_params_t* mode_params =
        (lora_protocol_mode_params_t*)(reply_buffer + sizeof(lora_protocol_header_t));

    uint8_t   chip_mode = 0;
    esp_err_t res       = sx126x_get_status(&lora_handle, NULL, &chip_mode);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LoRa chip mode: %s", esp_err_to_name(res));
        send_nack(sequence_number);
        return;
    }

    switch (chip_mode) {
        case SX126X_CHIP_MODE_STDBY_RC:
            mode_params->mode = LORA_PROTOCOL_MODE_STANDBY_RC;
            break;
        case SX126X_CHIP_MODE_STDBY_XOSC:
            mode_params->mode = LORA_PROTOCOL_MODE_STANDBY_XOSC;
            break;
        case SX126X_CHIP_MODE_FS:
            mode_params->mode = LORA_PROTOCOL_MODE_FS;
            break;
        case SX126X_CHIP_MODE_TX:
            mode_params->mode = LORA_PROTOCOL_MODE_TX;
            break;
        case SX126X_CHIP_MODE_RX:
            mode_params->mode = LORA_PROTOCOL_MODE_RX;
            break;
        default:
            ESP_LOGW(TAG, "Unknown LoRa chip mode: %d", chip_mode);
            mode_params->mode = LORA_PROTOCOL_MODE_UNKNOWN;
            break;
    }

    size_t mode_length = sizeof(lora_protocol_header_t) + sizeof(lora_protocol_mode_params_t);
    generate_custom_event(ESP_PRIV_EVENT_LORA, reply_buffer, mode_length);
}

esp_err_t apply_mode(uint8_t* mode_data, size_t mode_length) {
    if (mode_length < sizeof(lora_protocol_mode_params_t)) {
        ESP_LOGW(TAG, "SET_MODE command received with insufficient data length: %zu bytes", mode_length);
        return ESP_ERR_INVALID_SIZE;
    }

    lora_protocol_mode_params_t* mode_params = (lora_protocol_mode_params_t*)mode_data;

    ESP_LOGI(TAG, "Setting LoRa mode to %d", mode_params->mode);

    switch (mode_params->mode) {
        case LORA_PROTOCOL_MODE_STANDBY_RC:
            return sx126x_set_op_mode_standby(&lora_handle, false);
        case LORA_PROTOCOL_MODE_STANDBY_XOSC:
            return sx126x_set_op_mode_standby(&lora_handle, true);
        case LORA_PROTOCOL_MODE_FS:
            return sx126x_set_op_mode_fs(&lora_handle);
        case LORA_PROTOCOL_MODE_TX:
            return sx126x_set_op_mode_tx(&lora_handle);
        case LORA_PROTOCOL_MODE_RX:
            return sx126x_set_op_mode_rx(&lora_handle);
        default:
            ESP_LOGW(TAG, "Unknown LoRa mode requested: %d", mode_params->mode);
            return ESP_ERR_INVALID_ARG;
    }
}

static void send_config(uint32_t sequence_number) {
    lora_protocol_header_t* config_packet = (lora_protocol_header_t*)reply_buffer;
    config_packet->sequence_number        = sequence_number;
    config_packet->type                   = LORA_PROTOCOL_TYPE_GET_CONFIG;
    lora_protocol_config_params_t* config_params =
        (lora_protocol_config_params_t*)(reply_buffer + sizeof(lora_protocol_header_t));

    memcpy(config_params, &current_config, sizeof(lora_protocol_config_params_t));

    size_t config_length = sizeof(lora_protocol_header_t) + sizeof(lora_protocol_config_params_t);
    generate_custom_event(ESP_PRIV_EVENT_LORA, reply_buffer, config_length);
}

static esp_err_t apply_config(uint8_t* config_data, size_t config_length) {
    if (config_length < sizeof(lora_protocol_config_params_t)) {
        ESP_LOGW(TAG, "SET_CONFIG command received with insufficient data length: %zu bytes", config_length);
        return ESP_ERR_INVALID_SIZE;
    }

    lora_protocol_config_params_t* config_params = (lora_protocol_config_params_t*)config_data;

    // Here you would add code to apply the configuration to the LoRa module
    ESP_LOGI(TAG, "Applying LoRa configuration: Frequency=%.2f MHz, SF=%d, BW=%d kHz", config_params->frequency,
             config_params->spreading_factor, config_params->bandwidth);

    memcpy(&current_config, config_params, sizeof(lora_protocol_config_params_t));

    esp_err_t res = sx126x_set_regulator_mode(&lora_handle, true);  // Use DC-DC
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa regulator mode to DC-DC: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_rf_frequency(&lora_handle, config_params->frequency);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa RF frequency: %s", esp_err_to_name(res));
        return res;
    }

    sx126x_lora_spreading_factor_t spreading_factor;
    switch (config_params->spreading_factor) {
        case 5:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_5;
            break;
        case 6:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_6;
            break;
        case 7:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_7;
            break;
        case 8:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_8;
            break;
        case 9:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_9;
            break;
        case 10:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_10;
            break;
        case 11:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_11;
            break;
        case 12:
            spreading_factor = SX126X_LORA_SPREADING_FACTOR_12;
            break;
        default:
            ESP_LOGW(TAG, "Invalid spreading factor: %d", config_params->spreading_factor);
            return ESP_ERR_INVALID_ARG;
    }

    sx126x_lora_bandwidth_t bandwidth;
    switch (config_params->bandwidth) {
        case 7:
            bandwidth = SX126X_LORA_BANDWIDTH_7;
            break;
        case 10:
            bandwidth = SX126X_LORA_BANDWIDTH_10;
            break;
        case 15:
            bandwidth = SX126X_LORA_BANDWIDTH_15;
            break;
        case 20:
            bandwidth = SX126X_LORA_BANDWIDTH_20;
            break;
        case 31:
            bandwidth = SX126X_LORA_BANDWIDTH_31;
            break;
        case 41:
            bandwidth = SX126X_LORA_BANDWIDTH_41;
            break;
        case 62:
            bandwidth = SX126X_LORA_BANDWIDTH_62;
            break;
        case 125:
            bandwidth = SX126X_LORA_BANDWIDTH_125;
            break;
        case 250:
            bandwidth = SX126X_LORA_BANDWIDTH_250;
            break;
        case 500:
            bandwidth = SX126X_LORA_BANDWIDTH_500;
            break;
        default:
            ESP_LOGW(TAG, "Invalid bandwidth: %d kHz", config_params->bandwidth);
            return ESP_ERR_INVALID_ARG;
    }

    sx126x_lora_coding_rate_t coding_rate;
    switch (config_params->coding_rate) {
        case 5:
            coding_rate = SX126X_LORA_CODING_RATE_4_5;
            break;
        case 6:
            coding_rate = SX126X_LORA_CODING_RATE_4_6;
            break;
        case 7:
            coding_rate = SX126X_LORA_CODING_RATE_4_7;
            break;
        case 8:
            coding_rate = SX126X_LORA_CODING_RATE_4_8;
            break;
        default:
            ESP_LOGW(TAG, "Invalid coding rate: 4/%d", config_params->coding_rate);
            return ESP_ERR_INVALID_ARG;
    }

    res = sx126x_set_modulation_params_lora(&lora_handle, spreading_factor, bandwidth, coding_rate,
                                            config_params->low_data_rate_optimization);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa modulation parameters: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_packet_params_lora_variable_length(&lora_handle, config_params->preamble_length,
                                                        config_params->crc_enabled, config_params->invert_iq);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa packet parameters: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_sync_word(&lora_handle, config_params->sync_word);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa sync word and control bits: %s", esp_err_to_name(res));
        return res;
    }

    uint8_t pa_duty_cycle = 0x04;
    uint8_t hp_max        = 0x07;   // +22 dBm
    bool    is_sx1261     = false;  // True for SX1261, false for SX1262 and SX1268

    res = sx126x_set_pa_config(&lora_handle, pa_duty_cycle, hp_max, is_sx1261);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa PA configuration: %s", esp_err_to_name(res));
        return res;
    }

    bool pa_is_high_power = true;

    res = sx126x_set_tx_params(&lora_handle, config_params->power, pa_is_high_power, config_params->ramp_time);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa TX parameters: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_buffer_base_address(&lora_handle, 0x00, 0x00);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa buffer base address: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_stop_timer_on_preamble(&lora_handle, false);  // Stop timer on syncword
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LoRa stop timer on preamble: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_lora_symb_num_timeout(&lora_handle, 0);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa symbol number timeout: %s", esp_err_to_name(res));
        return res;
    }

    return ESP_OK;
}

static void send_status(uint32_t sequence_number) {
    lora_protocol_header_t* status_packet = (lora_protocol_header_t*)reply_buffer;
    status_packet->sequence_number        = sequence_number;
    status_packet->type                   = LORA_PROTOCOL_TYPE_GET_STATUS;
    lora_protocol_status_params_t* status_params =
        (lora_protocol_status_params_t*)(reply_buffer + sizeof(lora_protocol_header_t));

    // For demonstration purposes, we fill in some dummy status values
    status_params->errors    = 0;
    status_params->chip_type = LORA_PROTOCOL_CHIP_SX1262;
    snprintf(status_params->version_string, LORA_PROTOCOL_VERSION_STRING_LENGTH, "FIXME");

    size_t status_length = sizeof(lora_protocol_header_t) + sizeof(lora_protocol_status_params_t);
    generate_custom_event(ESP_PRIV_EVENT_LORA, reply_buffer, status_length);
}

static void send_packet(uint32_t sequence_number) {
    lora_protocol_header_t* packet = (lora_protocol_header_t*)reply_buffer;
    packet->sequence_number        = sequence_number;
    packet->type                   = LORA_PROTOCOL_TYPE_PACKET_RX;
    lora_protocol_lora_packet_t* lora_packet =
        (lora_protocol_lora_packet_t*)(reply_buffer + sizeof(lora_protocol_header_t));

    // For demonstration purposes, we fill in some dummy packet data
    const char* demo_data = "Hello, LoRa!";
    lora_packet->length   = strlen(demo_data);
    memcpy(lora_packet->data, demo_data, lora_packet->length);

    size_t packet_length = sizeof(lora_protocol_header_t) + sizeof(uint8_t) + lora_packet->length;
    generate_custom_event(ESP_PRIV_EVENT_LORA, reply_buffer, packet_length);
}

static esp_err_t transmit_packet(uint8_t* packet_data, size_t packet_length) {
    if (packet_length < sizeof(lora_protocol_lora_packet_t)) {
        ESP_LOGW(TAG, "PACKET_TX command received with insufficient data length: %zu bytes", packet_length);
        return ESP_ERR_INVALID_SIZE;
    }

    lora_protocol_lora_packet_t* lora_packet = (lora_protocol_lora_packet_t*)packet_data;

    // Here you would add code to transmit the packet via the LoRa module
    ESP_LOGI(TAG, "Transmitting LoRa packet of length %d", lora_packet->length);

    return ESP_OK;
}

void lora_protocol_handle_packet(uint8_t* request_buffer, size_t request_length) {
    printf("LoRa command: ");
    for (size_t i = 0; i < request_length; i++) {
        printf("%02X ", request_buffer[i]);
    }
    printf("\r\n");

    if (request_length < sizeof(lora_protocol_header_t)) {
        ESP_LOGW(TAG, "Received LoRa protocol packet is too short: %zu bytes", request_length);
        send_nack(0);
        return;
    }

    lora_protocol_header_t* packet = (lora_protocol_header_t*)request_buffer;

    switch (packet->type) {
        case LORA_PROTOCOL_TYPE_GET_MODE: {
            send_mode(packet->sequence_number);
            break;
        }
        case LORA_PROTOCOL_TYPE_SET_MODE:
            if (apply_mode(request_buffer + sizeof(lora_protocol_header_t),
                           request_length - sizeof(lora_protocol_header_t)) == ESP_OK) {
                send_ack(packet->sequence_number);
            } else {
                send_nack(packet->sequence_number);
            }
            break;
        case LORA_PROTOCOL_TYPE_GET_CONFIG:
            send_config(packet->sequence_number);
            break;
        case LORA_PROTOCOL_TYPE_SET_CONFIG:
            if (apply_config(request_buffer + sizeof(lora_protocol_header_t),
                             request_length - sizeof(lora_protocol_header_t)) == ESP_OK) {
                send_ack(packet->sequence_number);
            } else {
                send_nack(packet->sequence_number);
            }
            break;
        case LORA_PROTOCOL_TYPE_GET_STATUS:
            send_status(packet->sequence_number);
            break;
        case LORA_PROTOCOL_TYPE_PACKET_RX:
            send_packet(packet->sequence_number);
            break;
        case LORA_PROTOCOL_TYPE_PACKET_TX:
            if (transmit_packet(request_buffer + sizeof(lora_protocol_header_t),
                                request_length - sizeof(lora_protocol_header_t)) == ESP_OK) {
                send_ack(packet->sequence_number);
            } else {
                send_nack(packet->sequence_number);
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown command: %d\r\n", packet->type);
            send_nack(packet->sequence_number);
            break;
    }

    return;
}

esp_err_t lora_initialize(void) {
    esp_err_t res;

    res = sx126x_init(&lora_handle, BSP_LORA_BUS, BSP_LORA_CS, BSP_LORA_RESET, BSP_LORA_DIO1, BSP_LORA_BUSY);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LoRa radio: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_op_mode_standby(&lora_handle, false);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa radio to standby mode: %s", esp_err_to_name(res));
        return res;
    }

    char version_string[17] = {0};
    res                     = sx126x_read_version_string(&lora_handle, version_string, sizeof(version_string));
    if (res != ESP_OK) {
        return res;
    }

    res = sx126x_clear_device_errors(&lora_handle, NULL, NULL);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear LoRa radio device errors: %s", esp_err_to_name(res));
        return res;
    }

    const uint8_t testdata[] = {0x13, 0x37};
    res                      = sx126x_write_buffer(&lora_handle, 0x00, testdata, sizeof(testdata));
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data to LoRa radio: %s", esp_err_to_name(res));
        return res;
    }

    uint8_t read_data[sizeof(testdata)] = {0};
    res                                 = sx126x_read_buffer(&lora_handle, 0x00, read_data, sizeof(read_data));
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from LoRa radio: %s", esp_err_to_name(res));
        return res;
    }

    if (memcmp(testdata, read_data, sizeof(testdata)) != 0) {
        ESP_LOGE(TAG, "LoRa radio initialization failed: Read data does not match written data %02x %02x", read_data[0],
                 read_data[1]);
        return ESP_FAIL;
    }

    res = sx126x_set_dio2_as_rf_switch_ctrl(&lora_handle, true);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LoRa radio DIO2 as RF switch control: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_packet_type(&lora_handle, SX126X_PACKET_TYPE_LORA);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa radio packet type: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_rx_tx_fallback_mode(&lora_handle, SX126X_FALLBACK_MODE_STDBY_RC);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa radio RX/TX fallback mode: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_set_cad_params(&lora_handle, SX126X_CAD_ON_8_SYMB, 8 + 13, 10, false, 0xFFFFFF);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa radio CAD parameters: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_clear_irq_status(&lora_handle, 0);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear LoRa radio IRQ status: %s", esp_err_to_name(res));
        return res;
    }

    res = sx126x_calibrate(&lora_handle, true, true, true, true, true, true, true);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate LoRa radio: %s", esp_err_to_name(res));
        return res;
    }

    while (sx126x_is_busy(&lora_handle)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "Waiting for LoRa radio to be calibrated...");
    }

    uint16_t errors = 0;

    res = sx126x_get_device_errors(&lora_handle, &errors);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LoRa radio device errors: %s", esp_err_to_name(res));
        return res;
    }

    bool tcxo_detected = false;

    if (errors == SX126X_XOSC_START_ERR) {
        tcxo_detected = true;
        res           = sx126x_clear_device_errors(&lora_handle, NULL, NULL);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear LoRa radio device errors: %s", esp_err_to_name(res));
            return res;
        }
        res = sx126x_set_dio3_as_txco_ctrl(&lora_handle, 1.8, 5000);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LoRa radio DIO3 as TCXO control: %s", esp_err_to_name(res));
            return res;
        }
        res = sx126x_calibrate(&lora_handle, true, true, true, true, true, true, true);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to calibrate LoRa radio: %s", esp_err_to_name(res));
            return res;
        }
        while (sx126x_is_busy(&lora_handle)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_LOGI(TAG, "Waiting for LoRa radio to be calibrated...");
        }
        res = sx126x_get_device_errors(&lora_handle, &errors);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get LoRa radio device errors: %s", esp_err_to_name(res));
            return res;
        }
    }

    if (errors != 0) {
        ESP_LOGE(TAG, "LoRa radio device errors detected: 0x%04x", errors);

        if (errors & SX126X_ERROR_RC64K_CALIB_ERR) ESP_LOGE(TAG, "RC64K calibration error");
        if (errors & SX126X_ERROR_RC13M_CALIB_ERR) ESP_LOGE(TAG, "RC13M calibration error");
        if (errors & SX126X_ERROR_PLL_CALIB_ERR) ESP_LOGE(TAG, "PLL calibration error");
        if (errors & SX126X_ERROR_ADC_CALIB_ERR) ESP_LOGE(TAG, "ADC calibration error");
        if (errors & SX126X_ERROR_IMG_CALIB_ERR) ESP_LOGE(TAG, "Image calibration error");
        if (errors & SX126X_XOSC_START_ERR) ESP_LOGE(TAG, "XOSC start error");
        if (errors & SX126X_PLL_LOCK_ERR) ESP_LOGE(TAG, "PLL lock error");
        if (errors & SX126X_PA_RAMP_ERR) ESP_LOGE(TAG, "PA ramp error");
        return ESP_FAIL;
    }

    uint8_t chip_mode = 0;
    res               = sx126x_get_status(&lora_handle, NULL, &chip_mode);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LoRa radio status: %s", esp_err_to_name(res));
        return res;
    }

    if (chip_mode != SX126X_CHIP_MODE_STDBY_RC) {
        ESP_LOGE(TAG, "LoRa radio not in expected standby RC mode after initialization, mode: %d", chip_mode);
        return ESP_FAIL;
    }

    uint16_t irq_mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_PREAMBLE_DETECTED |
                        SX126X_IRQ_HEADER_VALID | SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_CAD_DONE |
                        SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_TIMEOUT;
    uint16_t dio1_mask = SX126X_IRQ_ALL;
    uint16_t dio2_mask = 0;  // DIO2 is used as RF switch control
    uint16_t dio3_mask = 0;  // DIO3 is used as TCXO control if applicable

    res = sx126x_set_dio_irq_params(&lora_handle, irq_mask, dio1_mask, dio2_mask, dio3_mask);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa DIO IRQ parameters: %s", esp_err_to_name(res));
        return res;
    }

    ESP_LOGI(TAG, "LoRa chip initialized (%s with %s)", version_string, tcxo_detected ? "TCXO" : "XOSC");

    return ESP_OK;
}

void read_data(void) {
    uint8_t   packet_length = 0;
    uint8_t   start_pointer = 0;
    esp_err_t res           = sx126x_get_rx_buffer_status(&lora_handle, &packet_length, &start_pointer);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LoRa RX buffer status: %s", esp_err_to_name(res));
        return;
    }
    if (packet_length == 0) {
        ESP_LOGW(TAG, "No data?");
        return;
    }

    uint8_t                 data[sizeof(lora_protocol_header_t) + 256] = {0};
    lora_protocol_header_t* lora_packet                                = (lora_protocol_header_t*)data;
    uint8_t*                packet                                     = &data[sizeof(lora_protocol_header_t)];
    res = sx126x_read_buffer(&lora_handle, start_pointer, packet, packet_length);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read packet from LoRa radio: %s", esp_err_to_name(res));
        return;
    }

    printf("Packet: ");
    for (size_t i = 0; i < packet_length; i++) {
        printf("%02X ", packet[i]);
    }
    printf("\r\n");

    lora_packet->sequence_number = 0;  // Sequence number is not used
    lora_packet->type            = LORA_PROTOCOL_TYPE_PACKET_RX;

    size_t total_length = sizeof(lora_protocol_header_t) + packet_length;
    generate_custom_event(ESP_PRIV_EVENT_LORA, data, total_length);
}

void lora_task(void* pvParameters) {
    esp_err_t res;

    res = lora_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "LoRa initialization failed: %s", esp_err_to_name(res));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "LoRa task started");

    while (1) {
        res = sx126x_irq_wait(&lora_handle, portMAX_DELAY);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Waiting for interrupt failed: %s", esp_err_to_name(res));
        }

        uint16_t interrupts     = 0;
        uint8_t  command_status = 0;
        uint8_t  chip_mode      = 0;
        res                     = sx126x_get_irq_status(&lora_handle, &interrupts, &command_status, &chip_mode);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get LoRa IRQ status: %s", esp_err_to_name(res));
            return;
        }

        if (interrupts & SX126X_IRQ_TX_DONE) printf("Interrupt: TX done\r\n");
        if (interrupts & SX126X_IRQ_RX_DONE) printf("Interrupt: RX done\r\n");
        if (interrupts & SX126X_IRQ_PREAMBLE_DETECTED) printf("Interrupt: preamble detected\r\n");
        if (interrupts & SX126X_IRQ_SYNC_WORD_VALID) printf("Interrupt: sync word valid\r\n");
        if (interrupts & SX126X_IRQ_HEADER_VALID) printf("Interrupt: header valid\r\n");
        if (interrupts & SX126X_IRQ_HEADER_ERROR) printf("Interrupt: header error\r\n");
        if (interrupts & SX126X_IRQ_CRC_ERROR) printf("Interrupt: crc error\r\n");
        if (interrupts & SX126X_IRQ_CAD_DONE) printf("Interrupt: cad done\r\n");
        if (interrupts & SX126X_IRQ_CAD_DETECTED) printf("Interrupt: cad detected\r\n");
        if (interrupts & SX126X_IRQ_TIMEOUT) printf("Interrupt: timeout\r\n");
        if (interrupts & SX126X_IRQ_LRFHSSHOP) printf("Interrupt: lrhsshop\r\n");

        res = sx126x_clear_irq_status(&lora_handle, SX126X_IRQ_ALL);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear LoRa radio IRQ status: %s", esp_err_to_name(res));
            return;
        }

        switch (command_status) {
            case SX126X_COMMAND_STATUS_DATA_AVAILABLE:
                printf("Data available!\r\n");
                read_data();

                res = sx126x_set_op_mode_rx(&lora_handle);
                if (res != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set LoRa radio to RX mode: %s", esp_err_to_name(res));
                    return;
                }
                break;
            case SX126X_COMMAND_STATUS_TIMEOUT:
                printf("Operation timed out!\r\n");
                break;
            case SX126X_COMMAND_STATUS_INVALID:
                printf("Invalid operation!\r\n");
                break;
            case SX126X_COMMAND_STATUS_FAILED:
                printf("Operation failed!\r\n");
                break;
            case SX126X_COMMAND_STATUS_TX_DONE:
                printf("Transmission done!\r\n");
                break;
            default:
                break;
        }

        switch (chip_mode) {
            case SX126X_CHIP_MODE_STDBY_RC:
                printf("Chip in STDBY_RC mode\r\n");
                break;
            case SX126X_CHIP_MODE_STDBY_XOSC:
                printf("Chip in STDBY_XOSC mode\r\n");
                break;
            case SX126X_CHIP_MODE_FS:
                printf("Chip in FS mode\r\n");
                break;
            case SX126X_CHIP_MODE_TX:
                printf("Chip in TX mode\r\n");
                break;
            case SX126X_CHIP_MODE_RX:
                printf("Chip in RX mode\r\n");
                break;
            default:
                break;
        }

        uint16_t errors = 0;
        res             = sx126x_get_device_errors(&lora_handle, &errors);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get LoRa radio device errors: %s", esp_err_to_name(res));
            return;
        }

        if (errors != 0) {
            ESP_LOGE(TAG, "LoRa radio device errors detected: 0x%04x", errors);

            if (errors & SX126X_ERROR_RC64K_CALIB_ERR) ESP_LOGE(TAG, "RC64K calibration error");
            if (errors & SX126X_ERROR_RC13M_CALIB_ERR) ESP_LOGE(TAG, "RC13M calibration error");
            if (errors & SX126X_ERROR_PLL_CALIB_ERR) ESP_LOGE(TAG, "PLL calibration error");
            if (errors & SX126X_ERROR_ADC_CALIB_ERR) ESP_LOGE(TAG, "ADC calibration error");
            if (errors & SX126X_ERROR_IMG_CALIB_ERR) ESP_LOGE(TAG, "Image calibration error");
            if (errors & SX126X_XOSC_START_ERR) ESP_LOGE(TAG, "XOSC start error");
            if (errors & SX126X_PLL_LOCK_ERR) ESP_LOGE(TAG, "PLL lock error");
            if (errors & SX126X_PA_RAMP_ERR) ESP_LOGE(TAG, "PA ramp error");
            return;
        }
    }

    vTaskDelete(NULL);
}

void start_lora_task() {
    xTaskCreatePinnedToCore(lora_task, "lora_task", 4096, NULL, 10, NULL, CONFIG_SOC_CPU_CORES_NUM - 1);
}
