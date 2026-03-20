#include "ir_protocol_server.h"
#include <stdio.h>
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_hosted_peer_data.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "interface.h"
#include "ir_nec_encoder.h"
#include "priv_events.h"
#include "sdio_slave_api.h"
#include "tanmatsu_hardware.h"

static const char*          TAG         = "ir";
static rmt_channel_handle_t tx_channel  = NULL;
static rmt_encoder_handle_t nec_encoder = NULL;

static esp_err_t init_ir(void) {
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = 1000000,  // 1MHz, 1 tick = 1us
        .mem_block_symbols = 64,       // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't
                                 // queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num          = BSP_IR_TX,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));

    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle   = 0.33,
        .frequency_hz = 38000,  // 38KHz
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));

    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = 1000000,  // 1MHz, 1 tick = 1us
    };

    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder));
    ESP_ERROR_CHECK(rmt_enable(tx_channel));
    return ESP_OK;
}

static void ir_protocol_packet_callback(uint32_t msg_id, const uint8_t* data, size_t data_len) {
    if (msg_id != TANMATSU_EVENT_IR) {
        ESP_LOGW(TAG, "Received message with unexpected ID: %d", msg_id);
        return;
    }

    if (data_len != sizeof(ir_nec_scan_code_t)) {
        ESP_LOGW(TAG, "Received IR data with unexpected length: %d", data_len);
        return;
    }

    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,  // no loop
    };

    const ir_nec_scan_code_t* scan_code = (const ir_nec_scan_code_t*)data;
    ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, scan_code, sizeof(ir_nec_scan_code_t), &transmit_config));
}

esp_err_t ir_initialize(void) {
    esp_err_t res;

    res = init_ir();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IR transmitter: %s", esp_err_to_name(res));
        return res;
    }

    res = esp_hosted_register_custom_callback(TANMATSU_EVENT_IR, ir_protocol_packet_callback);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IR protocol callback: %s", esp_err_to_name(res));
        return res;
    }

    return ESP_OK;
}
