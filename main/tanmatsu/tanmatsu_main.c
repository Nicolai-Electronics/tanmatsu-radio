#include "badgelink_protocol_server.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "echo_protocol_server.h"
#include "esp_err.h"
#include "esp_hosted_coprocessor.h"
#include "esp_hosted_peer_data.h"
#include "esp_log.h"
#include "ieee802154_protocol_server.h"
#include "ir_protocol_server.h"
#include "lora_protocol_server.h"
#include "nvs_flash.h"
#include "priv_events.h"
#include "system_protocol_server.h"
#include "tanmatsu_hardware.h"
#include "ulp_functions.h"

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

    res = echo_protocol_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize echo protocol: %s", esp_err_to_name(res));
        // (ignore errors, continue)
    }

    res = infrared_protocol_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize infrared protocol: %s", esp_err_to_name(res));
        // (ignore errors, continue)
    }

    res = badgelink_protocol_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize badgelink protocol: %s", esp_err_to_name(res));
        // (ignore errors, continue)
    }

    res = system_protocol_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize system protocol: %s", esp_err_to_name(res));
        // (ignore errors, continue)
    }

    res = ieee802154_protocol_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IEEE802.15.4 protocol: %s", esp_err_to_name(res));
        // (ignore errors, continue)
    }

    res = spi_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(res));
        return;  // Can't start LoRa without working SPI bus
    }

    res = lora_protocol_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LoRa protocol: %s", esp_err_to_name(res));
        // (ignore errors, continue)
    }

    ulp_init();
}
