#include "airplane_mode.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_err.h"
#include "esp_hosted_coprocessor.h"
#include "esp_hosted_peer_data.h"
#include "esp_log.h"
#include "ir_protocol_server.h"
#include "lora_protocol_server.h"
#include "nvs_flash.h"
#include "priv_events.h"
#include "tanmatsu_hardware.h"

static const char* TAG = "tanmatsu";

// SPI bus

static esp_err_t spi_initialize(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num   = BSP_LORA_MISO,
        .mosi_io_num   = BSP_LORA_MOSI,
        .sclk_io_num   = BSP_LORA_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    return spi_bus_initialize(BSP_LORA_BUS, &buscfg, SPI_DMA_CH_AUTO);
}

// Echo

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

// Main

void app_main(void) {
    esp_err_t res = nvs_flash_init();

    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        res = nvs_flash_init();
    }
    ESP_ERROR_CHECK(res);

    gpio_install_isr_service(0);

    esp_hosted_coprocessor_init();

    res = airplane_mode_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize airplane mode: %s", esp_err_to_name(res));
        // (ignore errors, continue)
    }

    res = esp_hosted_register_custom_callback(TANMATSU_EVENT_ECHO, echo_protocol_packet_callback);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register echo protocol callback: %s", esp_err_to_name(res));
        // (ignore errors, continue)
    }

    res = ir_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IR: %s", esp_err_to_name(res));
        // (ignore errors, continue)
    }

    res = spi_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(res));
        return;  // Can't start LoRa without working SPI bus
    }

    start_lora_task();
}
