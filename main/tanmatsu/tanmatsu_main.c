#include "badgelink_protocol_server.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "echo_protocol_server.h"
#include "esp_err.h"
#include "esp_hosted_coprocessor.h"
#include "esp_hosted_peer_data.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "ieee802154_protocol_server.h"
#include "ir_protocol_server.h"
#include "lora_protocol_server.h"
#include "nvs_flash.h"
#include "priv_events.h"
#include "system_protocol_server.h"

#include "hardware.h"

#ifdef DEVICE_IS_WHY2025
#define PWM_TIMER_BASE_CLK LEDC_USE_XTAL_CLK
#define PWM_TIMER_RESOLUTION LEDC_TIMER_8_BIT
#define PWM_TIMER_FREQ_HZ 25000  // Set well above hearing frequency

#include "driver/ledc.h"
void configure_pwm_timer(ledc_timer_t timer_num)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_TIMER_RESOLUTION,
        .timer_num = timer_num,
        .freq_hz = PWM_TIMER_FREQ_HZ,
        .clk_cfg = PWM_TIMER_BASE_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));
}

void configure_pwm(int gpio_num, ledc_channel_t channel, ledc_timer_t timer_num) {
    ledc_channel_config_t channel_conf = {
        .gpio_num = gpio_num,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer_num,
        .duty = 0, // Initial duty cycle
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

/// @brief set the pwm value and limit the duty to [0, 10-80]
/// @param channel
/// @param duty_percentage
void set_pwm_duty_cycle(ledc_channel_t channel, int duty_percentage) {
    uint32_t duty = (uint32_t)((1 << (PWM_TIMER_RESOLUTION)) * duty_percentage / 100);
    // limit duty % to to 0, 10-80
    if (duty < 10 && duty != 0)
    {
        duty = 10;
    }
    if (duty > 80)
    {
        duty = 80;
    }
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

#endif

#include "ulp_functions.h"

static const char* TAG = "tanmatsu";

uint8_t protocol_server_reply_buffer[512];

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

#if DEVICE_IS_WHY2025
    configure_pwm_timer(LEDC_TIMER_0);
    // Configure PWM on GPIO 15, using timer 0, channel 0
    configure_pwm(BSP_DISPLAY_BL, LEDC_CHANNEL_0, LEDC_TIMER_0);
    // Set duty cycle to 10%
    set_pwm_duty_cycle(LEDC_CHANNEL_0, 10);

    // Configure PWM on GPIO 10, using timer 0, channel 1
    configure_pwm(BSP_KEYBOARD_BL, LEDC_CHANNEL_1, LEDC_TIMER_0);
    // Set duty cycle to 50%
    set_pwm_duty_cycle(LEDC_CHANNEL_1, 10);
#endif

    gpio_install_isr_service(0);

    esp_hosted_coprocessor_init();

    res = echo_protocol_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize echo protocol: %s", esp_err_to_name(res));
    }

    res = infrared_protocol_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize infrared protocol: %s", esp_err_to_name(res));
    }

    res = badgelink_protocol_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize badgelink protocol: %s", esp_err_to_name(res));
    }

    res = system_protocol_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize system protocol: %s", esp_err_to_name(res));
    }

    res = ieee802154_protocol_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IEEE802.15.4 protocol: %s", esp_err_to_name(res));
    }

    res = spi_initialize();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(res));
    } else {
        res = lora_protocol_initialize();
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize LoRa protocol: %s", esp_err_to_name(res));
        }
    }

    ulp_init();

    esp_log_level_set("slave_rpc", ESP_LOG_WARN);
}
