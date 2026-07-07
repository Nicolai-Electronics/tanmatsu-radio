#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tanmatsu_hardware.h"
#include "tanmatsu_ulp.h"
#include "ulp_lp_core.h"

extern const uint8_t tanmatsu_ulp_bin_start[] asm("_binary_tanmatsu_ulp_bin_start");
extern const uint8_t tanmatsu_ulp_bin_end[] asm("_binary_tanmatsu_ulp_bin_end");

static const char TAG[] = "ULP";

static void monitor_task(void* arg) {
    ESP_LOGI(TAG, "ULP monitor task started");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static esp_err_t ulp_load_and_run(void) {
    ulp_lp_core_cfg_t cfg = {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU,
    };

    esp_err_t res = ulp_lp_core_load_binary(tanmatsu_ulp_bin_start, (tanmatsu_ulp_bin_end - tanmatsu_ulp_bin_start));
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "ULP load failed");
        return res;
    }

    res = ulp_lp_core_run(&cfg);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "ULP run failed");
        return res;
    }

    ESP_LOGI(TAG, "ULP started");
    return ESP_OK;
}

esp_err_t ulp_init(void) {
    /*rtc_gpio_init(BSP_LORA_SCK);
    rtc_gpio_init(BSP_LORA_CS);
    rtc_gpio_init(BSP_LORA_MOSI);
    rtc_gpio_init(BSP_LORA_MISO);
    rtc_gpio_init(BSP_LORA_DIO1);
    rtc_gpio_init(BSP_LORA_BUSY);*/

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        ESP_LOGI(TAG, "HP CPU started from wakeup source other than ULP, starting ULP firmware...");
        esp_err_t res = ulp_load_and_run();
        if (res != ESP_OK) {
            return res;
        }
    } else if (cause == ESP_SLEEP_WAKEUP_ULP) {
        ESP_LOGI(TAG, "HP CPU started from ULP wakeup source, ULP firmware already running");
    }

    xTaskCreate(monitor_task, "ulp_monitor", 4096, NULL, 5, NULL);

    return esp_sleep_enable_ulp_wakeup();
}
